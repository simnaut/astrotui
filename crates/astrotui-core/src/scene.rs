//! The scene data model and ingestion API (DESIGN.md §4.1 / §4.2).
//!
//! astrotui-core owns the canonical render model, the [`SceneStore`]; producers populate
//! it **by value** through [`SceneWriter`] handles scoped to named **layers**. The widget
//! reads the latest committed [`Snapshot`] lock-free, and the rendered scene is the live
//! **union** of all layers — so each producer (a live sim, an ephemeris body-filler, a
//! telemetry feed) has independent lifecycle and cadence without clobbering the others.
//!
//! This module is the ingestion machinery (issue #12). Rich object metadata (label,
//! kind, shape, trail, path) is added in #13; building an astrodyn `FrameTree` from the
//! frame records and the render pass land in #14.

use std::collections::BTreeMap;
use std::fmt;
use std::sync::{Arc, Mutex};

use arc_swap::ArcSwap;
use astrodyn_quantities::{SecondsSince, TDB};
use glam::{DQuat, DVec3};

/// Epoch a sample is stamped with — TDB seconds, the wire-format epoch (DESIGN.md §4.3).
pub type Epoch = SecondsSince<TDB>;

macro_rules! string_id {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
        pub struct $name(Arc<str>);

        impl $name {
            /// The id as a string slice.
            #[must_use]
            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
        impl From<&str> for $name {
            fn from(s: &str) -> Self {
                Self(Arc::from(s))
            }
        }
        impl From<String> for $name {
            fn from(s: String) -> Self {
                Self(Arc::from(s))
            }
        }
        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }
    };
}

string_id!(
    /// Stable id of a frame node, as declared by a producer (e.g. `"moon_fixed"`).
    FrameId
);
string_id!(
    /// Stable id of a scene object, as declared by a producer (e.g. `"lander"`).
    ObjectId
);
string_id!(
    /// Name of a producer layer (e.g. `"sim"`, `"ephemeris"`, `"telemetry"`).
    LayerId
);

/// Kinematic state expressed in an object's (or frame's) **native frame** (DESIGN.md §4.2);
/// never pre-projected into the camera frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyState {
    /// Position in the native frame (metres).
    pub position: DVec3,
    /// Velocity in the native frame (m/s).
    pub velocity: DVec3,
    /// Orientation of the body axes within the native frame.
    pub attitude: DQuat,
}

impl Default for BodyState {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            attitude: DQuat::IDENTITY,
        }
    }
}

/// A frame node: its parent in the tree and its state relative to that parent.
/// `parent == None` marks the root frame.
#[derive(Clone, Debug)]
pub struct FrameRecord {
    /// This frame's stable id.
    pub id: FrameId,
    /// Parent frame id, or `None` for the root.
    pub parent: Option<FrameId>,
    /// State of this frame relative to its parent.
    pub state: BodyState,
}

/// An object placed on a frame, carrying its state in that frame.
///
/// Enriched with label/kind/shape/trail/path in #13; for now it is the kinematic record
/// the store ingests and the render pass (#14) will project.
#[derive(Clone, Debug)]
pub struct ObjectRecord {
    /// This object's stable id.
    pub id: ObjectId,
    /// The frame this object lives in.
    pub frame: FrameId,
    /// State in the object's native `frame`.
    pub state: BodyState,
}

/// One producer layer's committed content.
#[derive(Clone, Default)]
struct Layer {
    epoch: Option<Epoch>,
    frames: BTreeMap<FrameId, FrameRecord>,
    objects: BTreeMap<ObjectId, ObjectRecord>,
}

/// An immutable, point-in-time view of the whole scene: the union of every layer's
/// most recent commit. The widget renders from this and may hold it across a commit —
/// it never mutates, so a reader always sees a consistent scene.
#[derive(Debug, Default)]
pub struct Snapshot {
    frames: Vec<FrameRecord>,
    objects: Vec<ObjectRecord>,
    layer_epochs: Vec<(LayerId, Epoch)>,
}

impl Snapshot {
    /// All frame nodes across the union of layers (ascending by id).
    #[must_use]
    pub fn frames(&self) -> &[FrameRecord] {
        &self.frames
    }

    /// All objects across the union of layers (ascending by id).
    #[must_use]
    pub fn objects(&self) -> &[ObjectRecord] {
        &self.objects
    }

    /// The epoch of the layer's most recent commit, if that layer has committed.
    #[must_use]
    pub fn epoch(&self, layer: &LayerId) -> Option<Epoch> {
        self.layer_epochs
            .iter()
            .find(|(id, _)| id == layer)
            .map(|(_, e)| *e)
    }

    /// `true` when no layer has contributed any frames or objects.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty() && self.objects.is_empty()
    }
}

struct Inner {
    /// Authoritative per-layer content; writers serialize here (not the hot path).
    layers: Mutex<BTreeMap<LayerId, Layer>>,
    /// The published union snapshot, swapped atomically and read lock-free.
    published: ArcSwap<Snapshot>,
}

impl Inner {
    /// Rebuild the union snapshot from all layers. Caller holds the `layers` lock.
    ///
    /// Producers own disjoint id sets (DESIGN.md §4.1); if two layers nonetheless declare
    /// the same id, the layer later in `LayerId` order wins, deterministically.
    fn rebuild(layers: &BTreeMap<LayerId, Layer>) -> Snapshot {
        let mut frames: BTreeMap<FrameId, FrameRecord> = BTreeMap::new();
        let mut objects: BTreeMap<ObjectId, ObjectRecord> = BTreeMap::new();
        let mut layer_epochs = Vec::new();
        for (layer_id, layer) in layers {
            if let Some(epoch) = layer.epoch {
                layer_epochs.push((layer_id.clone(), epoch));
            }
            for (id, rec) in &layer.frames {
                frames.insert(id.clone(), rec.clone());
            }
            for (id, rec) in &layer.objects {
                objects.insert(id.clone(), rec.clone());
            }
        }
        Snapshot {
            frames: frames.into_values().collect(),
            objects: objects.into_values().collect(),
            layer_epochs,
        }
    }
}

/// The canonical, owned render model. The host creates one and keeps it for the app's
/// whole life; producers write into it via [`SceneStore::writer`], the widget reads it
/// via [`SceneStore::snapshot`].
#[derive(Clone)]
pub struct SceneStore {
    inner: Arc<Inner>,
}

impl Default for SceneStore {
    fn default() -> Self {
        Self::new()
    }
}

impl SceneStore {
    /// Create an empty store.
    #[must_use]
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Inner {
                layers: Mutex::new(BTreeMap::new()),
                published: ArcSwap::from_pointee(Snapshot::default()),
            }),
        }
    }

    /// Get a [`SceneWriter`] for a named layer. Multiple writers for the same layer name
    /// share that layer; different names are isolated. The handle is `Send + Clone`, so
    /// it can be moved to a producer thread.
    #[must_use]
    pub fn writer(&self, layer: impl Into<LayerId>) -> SceneWriter {
        SceneWriter {
            inner: Arc::clone(&self.inner),
            layer: layer.into(),
        }
    }

    /// The latest committed snapshot — a cheap, lock-free atomic load. Hold it as long as
    /// you like; it is immutable and unaffected by later commits.
    #[must_use]
    pub fn snapshot(&self) -> Arc<Snapshot> {
        self.inner.published.load_full()
    }
}

/// A `Send + Clone` handle that publishes into one named layer of a [`SceneStore`].
#[derive(Clone)]
pub struct SceneWriter {
    inner: Arc<Inner>,
    layer: LayerId,
}

impl SceneWriter {
    /// The layer this writer publishes into.
    #[must_use]
    pub fn layer(&self) -> &LayerId {
        &self.layer
    }

    /// Begin a transaction stamped with `epoch`. Stage frames/objects on it, then
    /// [`Transaction::commit`] to atomically replace this layer's contents.
    pub fn begin(&self, epoch: Epoch) -> Transaction {
        Transaction {
            inner: Arc::clone(&self.inner),
            layer_id: self.layer.clone(),
            staged: Layer {
                epoch: Some(epoch),
                ..Layer::default()
            },
        }
    }
}

/// A staged set of frames and objects for one layer. Building a transaction does not
/// touch the store; [`Transaction::commit`] publishes it as the layer's new contents.
#[must_use = "a Transaction does nothing until committed"]
pub struct Transaction {
    inner: Arc<Inner>,
    layer_id: LayerId,
    staged: Layer,
}

impl Transaction {
    /// Stage a frame node (its parent and state relative to that parent).
    pub fn frame(
        &mut self,
        id: impl Into<FrameId>,
        parent: Option<FrameId>,
        state: BodyState,
    ) -> &mut Self {
        let id = id.into();
        self.staged
            .frames
            .insert(id.clone(), FrameRecord { id, parent, state });
        self
    }

    /// Stage an object placed on `frame`, with `state` in that native frame.
    pub fn object(
        &mut self,
        id: impl Into<ObjectId>,
        frame: impl Into<FrameId>,
        state: BodyState,
    ) -> &mut Self {
        let id = id.into();
        self.staged.objects.insert(
            id.clone(),
            ObjectRecord {
                id,
                frame: frame.into(),
                state,
            },
        );
        self
    }

    /// Atomically replace this layer's contents with the staged set and republish the
    /// union snapshot. Other layers are untouched.
    pub fn commit(self) {
        // Recover from a poisoned lock rather than propagating the panic: a producer
        // panicking mid-commit must not permanently wedge a long-lived store. The guarded
        // data is only the layer map, and this commit fully replaces its layer and
        // rebuilds the union, so continuing from a poisoned state is safe.
        let mut layers = self
            .inner
            .layers
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        layers.insert(self.layer_id, self.staged);
        let snapshot = Inner::rebuild(&layers);
        self.inner.published.store(Arc::new(snapshot));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread;

    fn st(x: f64) -> BodyState {
        BodyState {
            position: DVec3::new(x, 0.0, 0.0),
            ..BodyState::default()
        }
    }
    fn ep(s: f64) -> Epoch {
        Epoch::from_seconds(s)
    }
    fn obj_ids(snap: &Snapshot) -> Vec<&str> {
        snap.objects().iter().map(|o| o.id.as_str()).collect()
    }
    // SecondsSince<TDB> isn't PartialEq (the TDB marker isn't), so compare raw seconds.
    fn secs(e: Option<Epoch>) -> Option<f64> {
        e.map(|e| e.as_seconds())
    }

    #[test]
    fn commit_publishes_frames_and_objects() {
        let store = SceneStore::new();
        assert!(store.snapshot().is_empty());

        let w = store.writer("sim");
        let mut tx = w.begin(ep(100.0));
        tx.frame("root", None, BodyState::default())
            .frame("moon_fixed", Some("root".into()), st(1.0))
            .object("lander", "moon_fixed", st(2.0));
        tx.commit();

        let snap = store.snapshot();
        assert_eq!(snap.frames().len(), 2);
        assert_eq!(obj_ids(&snap), ["lander"]);
        assert_eq!(secs(snap.epoch(&"sim".into())), Some(100.0));
    }

    #[test]
    fn layers_union_and_isolate() {
        let store = SceneStore::new();
        let mut tx = store.writer("sim").begin(ep(1.0));
        tx.object("lander", "moon_fixed", st(1.0));
        tx.commit();
        let mut tx = store.writer("ephemeris").begin(ep(2.0));
        tx.object("moon", "root", st(2.0))
            .object("earth", "root", st(3.0));
        tx.commit();

        let snap = store.snapshot();
        assert_eq!(obj_ids(&snap), ["earth", "lander", "moon"]); // union, sorted by id
        assert_eq!(secs(snap.epoch(&"sim".into())), Some(1.0));
        assert_eq!(secs(snap.epoch(&"ephemeris".into())), Some(2.0));

        // Re-committing "sim" replaces only that layer; "ephemeris" persists untouched.
        let mut tx = store.writer("sim").begin(ep(5.0));
        tx.object("orbiter", "moon_fixed", st(9.0));
        tx.commit();
        let snap = store.snapshot();
        assert_eq!(obj_ids(&snap), ["earth", "moon", "orbiter"]); // lander gone, bodies stay
        assert_eq!(secs(snap.epoch(&"ephemeris".into())), Some(2.0));
    }

    #[test]
    fn old_snapshot_is_immutable_across_commit() {
        let store = SceneStore::new();
        let mut tx = store.writer("sim").begin(ep(1.0));
        tx.object("a", "f", st(1.0));
        tx.commit();
        let old = store.snapshot();

        let mut tx = store.writer("sim").begin(ep(2.0));
        tx.object("a", "f", st(1.0)).object("b", "f", st(2.0));
        tx.commit();

        assert_eq!(obj_ids(&old), ["a"]); // previously-loaded snapshot unchanged
        let cur = store.snapshot();
        assert_eq!(obj_ids(&cur), ["a", "b"]);
    }

    #[test]
    fn writer_is_send_and_concurrent_reads_stay_consistent() {
        let store = SceneStore::new();
        let mut tx = store.writer("sim").begin(ep(0.0));
        tx.object("a", "f", st(0.0));
        tx.commit();

        let writer = store.writer("sim"); // Send + Clone, moved to the producer thread
        let stop = Arc::new(AtomicBool::new(false));
        let stop_w = Arc::clone(&stop);
        let handle = thread::spawn(move || {
            let mut n = 0u64;
            while !stop_w.load(Ordering::Relaxed) {
                let mut tx = writer.begin(ep(n as f64));
                tx.object("a", "f", st(0.0));
                if n.is_multiple_of(2) {
                    tx.object("b", "f", st(1.0));
                }
                tx.commit();
                n += 1;
            }
        });

        // Every snapshot the reader observes must be one of the two consistent states —
        // never a torn mix — proving the publish is atomic and reads are lock-free.
        for _ in 0..5_000 {
            let snap = store.snapshot();
            let ids = obj_ids(&snap);
            assert!(
                ids == ["a"] || ids == ["a", "b"],
                "torn snapshot observed: {ids:?}"
            );
        }
        stop.store(true, Ordering::Relaxed);
        handle.join().unwrap();
    }
}
