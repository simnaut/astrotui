//! The scene data model and ingestion API (DESIGN.md §4.1 / §4.2).
//!
//! astrotui-core owns the canonical render model, the [`SceneStore`]; producers populate
//! it **by value** through [`SceneWriter`] handles scoped to named **layers**. The widget
//! reads the latest committed [`Snapshot`] lock-free, and the rendered scene is the live
//! **union** of all layers — so each producer (a live sim, an ephemeris body-filler, a
//! telemetry feed) has independent lifecycle and cadence without clobbering the others.
//!
//! Frame identity is **`astrodyn_quantities::FrameUid`** (astrodyn #659; DESIGN §3): a
//! `FrameRecord` is keyed by its `uid`, and a [`SceneObject`]/[`crate::render::Camera`]
//! names its frame by that uid. The uid carries the frame's class/role/tag, so there is no
//! separate "kind" field — identity and classification are one value. Objects additionally
//! carry id/label/kind/state/shape/path. The rolling trail (DESIGN.md §4.2 `trail`) is
//! accumulated store-side and attaches in P2 (#24).

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::sync::{Arc, Mutex};

use arc_swap::ArcSwap;
use astrodyn_planet::PlanetShape;
use astrodyn_quantities::{FrameUid, SecondsSince, TDB};
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

/// A frame node: its identity, its parent in the tree, and its state relative to that
/// parent. `parent == None` marks a root frame. Identity is a `FrameUid` (astrodyn #659),
/// which carries the frame's class/role/tag — so the uid is both the handle (resolution /
/// dangling-detection) and the classification.
#[derive(Clone, Debug)]
pub struct FrameRecord {
    /// This frame's identity (carries class/role/tag).
    pub uid: FrameUid,
    /// Parent frame's identity, or `None` for a root.
    pub parent: Option<FrameUid>,
    /// Frame epoch: the time-validity of this state (DESIGN §4.3 / astrodyn RFS-603).
    pub epoch: Option<Epoch>,
    /// State of this frame relative to its parent.
    pub state: BodyState,
}

/// What an object is, for rendering and the camera/frame switcher UI (DESIGN.md §4.2).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ObjectKind {
    /// A celestial body (planet, moon, sun).
    Body,
    /// A spacecraft / vehicle.
    Spacecraft,
    /// A fixed surface site (landing site, ground station).
    Site,
    /// A generic point of interest.
    #[default]
    Marker,
}

/// An object's physical shape. Wraps an `astrodyn_planet::PlanetShape` ellipsoid; a DEM
/// terrain handle is added alongside the DEM pipeline in P2.
#[derive(Clone, Copy, Debug)]
pub struct BodyShape {
    /// The reference ellipsoid (mean radii, mu, flattening).
    pub ellipsoid: PlanetShape,
}

impl BodyShape {
    /// Wrap a `PlanetShape` ellipsoid.
    #[must_use]
    pub fn ellipsoid(shape: PlanetShape) -> Self {
        Self { ellipsoid: shape }
    }
}

/// A caller-supplied planned/future polyline, in the owning object's native frame
/// (DESIGN.md §7). Distinct from the rolling trail (accumulated past track), which lands
/// with its store-side wiring in P2 (#24).
#[derive(Clone, Debug, Default)]
pub struct Path {
    /// Polyline vertices in the object's native frame (metres).
    pub points: Vec<DVec3>,
}

/// Per-object render metadata supplied with a state on [`Transaction::object`] — the
/// `meta` of DESIGN.md §4.1's `tx.object(obj_id, frame_uid, state, meta)`.
#[derive(Clone, Debug, Default)]
pub struct ObjectMeta {
    /// Human label shown in the camera/frame switcher UI.
    pub label: Cow<'static, str>,
    /// What the object is.
    pub kind: ObjectKind,
    /// Optional physical shape (drives LOD: point → ellipsoid → DEM mesh).
    pub shape: Option<BodyShape>,
    /// Optional caller-supplied planned path.
    pub path: Option<Path>,
}

/// An object placed on a frame: its state in that native frame plus render metadata.
///
/// The rolling trail (`trail` in DESIGN.md §4.2) is accumulated store-side and attaches
/// in P2 (#24); everything else in the §4.2 object model is here.
#[derive(Clone, Debug)]
pub struct SceneObject {
    /// This object's stable id.
    pub id: ObjectId,
    /// Human label for UI enumeration.
    pub label: Cow<'static, str>,
    /// Identity of the frame this object lives in (astrodyn #659).
    pub frame: FrameUid,
    /// What the object is.
    pub kind: ObjectKind,
    /// State in the object's native `frame`.
    pub state: BodyState,
    /// Optional physical shape.
    pub shape: Option<BodyShape>,
    /// Optional caller-supplied planned path.
    pub path: Option<Path>,
}

/// One producer layer's committed content.
#[derive(Clone, Default)]
struct Layer {
    epoch: Option<Epoch>,
    /// Frame nodes keyed by identity. `FrameUid` is `Hash + Eq` but not `Ord`
    /// (`Tag` holds a `Box<str>`), so this is a `HashMap`, not a `BTreeMap`.
    frames: HashMap<FrameUid, FrameRecord>,
    objects: BTreeMap<ObjectId, SceneObject>,
}

/// An immutable, point-in-time view of the whole scene: the union of every layer's
/// most recent commit. The widget renders from this and may hold it across a commit —
/// it never mutates, so a reader always sees a consistent scene.
#[derive(Debug, Default)]
pub struct Snapshot {
    frames: Vec<FrameRecord>,
    objects: Vec<SceneObject>,
    layer_epochs: Vec<(LayerId, Epoch)>,
}

impl Snapshot {
    /// All frame nodes across the union of layers (ordered deterministically by uid).
    #[must_use]
    pub fn frames(&self) -> &[FrameRecord] {
        &self.frames
    }

    /// All objects across the union of layers (ascending by id).
    #[must_use]
    pub fn objects(&self) -> &[SceneObject] {
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
        let mut frames: HashMap<FrameUid, FrameRecord> = HashMap::new();
        let mut objects: BTreeMap<ObjectId, SceneObject> = BTreeMap::new();
        let mut layer_epochs = Vec::new();
        for (layer_id, layer) in layers {
            if let Some(epoch) = layer.epoch {
                layer_epochs.push((layer_id.clone(), epoch));
            }
            for (uid, rec) in &layer.frames {
                frames.insert(uid.clone(), rec.clone());
            }
            for (id, rec) in &layer.objects {
                objects.insert(id.clone(), rec.clone());
            }
        }
        // `FrameUid` isn't `Ord`, so sort the union by its `Display` for a deterministic
        // frame order (golden frames, stable diagnostics). Object order is `ObjectId`-sorted
        // by the `BTreeMap`.
        let mut frames: Vec<FrameRecord> = frames.into_values().collect();
        frames.sort_by(|a, b| a.uid.to_string().cmp(&b.uid.to_string()));
        Snapshot {
            frames,
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
    /// Stage a frame node by its `uid` (its parent and state relative to that parent). The
    /// frame's epoch is the transaction's epoch.
    pub fn frame(
        &mut self,
        uid: FrameUid,
        parent: Option<FrameUid>,
        state: BodyState,
    ) -> &mut Self {
        let epoch = self.staged.epoch;
        self.staged.frames.insert(
            uid.clone(),
            FrameRecord {
                uid,
                parent,
                epoch,
                state,
            },
        );
        self
    }

    /// Stage an object placed on the frame named by `frame` (a [`FrameUid`]), with `state`
    /// in that native frame and render `meta` — DESIGN.md §4.1's
    /// `tx.object(obj_id, frame_uid, state, meta)`.
    pub fn object(
        &mut self,
        id: impl Into<ObjectId>,
        frame: FrameUid,
        state: BodyState,
        meta: ObjectMeta,
    ) -> &mut Self {
        let id = id.into();
        self.staged.objects.insert(
            id.clone(),
            SceneObject {
                id,
                label: meta.label,
                frame,
                kind: meta.kind,
                state,
                shape: meta.shape,
                path: meta.path,
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
    use astrodyn_quantities::{Moon, PlanetFixed, RootInertial};
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
    // Two distinct, real frame identities for the store machinery tests.
    fn f_root() -> FrameUid {
        FrameUid::of::<RootInertial>()
    }
    fn f_moon() -> FrameUid {
        FrameUid::of::<PlanetFixed<Moon>>()
    }
    fn obj_ids(snap: &Snapshot) -> Vec<&str> {
        snap.objects().iter().map(|o| o.id.as_str()).collect()
    }
    // SecondsSince<TDB> isn't PartialEq (the TDB marker isn't), so compare raw seconds.
    fn secs(e: Option<Epoch>) -> Option<f64> {
        e.map(|e| e.as_seconds())
    }
    // Default metadata for tests that don't exercise it.
    fn m() -> ObjectMeta {
        ObjectMeta::default()
    }

    #[test]
    fn commit_publishes_frames_and_objects() {
        let store = SceneStore::new();
        assert!(store.snapshot().is_empty());

        let w = store.writer("sim");
        let mut tx = w.begin(ep(100.0));
        tx.frame(f_root(), None, BodyState::default())
            .frame(f_moon(), Some(f_root()), st(1.0))
            .object("lander", f_moon(), st(2.0), m());
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
        tx.object("lander", f_moon(), st(1.0), m());
        tx.commit();
        let mut tx = store.writer("ephemeris").begin(ep(2.0));
        tx.object("moon", f_root(), st(2.0), m())
            .object("earth", f_root(), st(3.0), m());
        tx.commit();

        let snap = store.snapshot();
        assert_eq!(obj_ids(&snap), ["earth", "lander", "moon"]); // union, sorted by id
        assert_eq!(secs(snap.epoch(&"sim".into())), Some(1.0));
        assert_eq!(secs(snap.epoch(&"ephemeris".into())), Some(2.0));

        // Re-committing "sim" replaces only that layer; "ephemeris" persists untouched.
        let mut tx = store.writer("sim").begin(ep(5.0));
        tx.object("orbiter", f_moon(), st(9.0), m());
        tx.commit();
        let snap = store.snapshot();
        assert_eq!(obj_ids(&snap), ["earth", "moon", "orbiter"]); // lander gone, bodies stay
        assert_eq!(secs(snap.epoch(&"ephemeris".into())), Some(2.0));
    }

    #[test]
    fn old_snapshot_is_immutable_across_commit() {
        let store = SceneStore::new();
        let mut tx = store.writer("sim").begin(ep(1.0));
        tx.object("a", f_root(), st(1.0), m());
        tx.commit();
        let old = store.snapshot();

        let mut tx = store.writer("sim").begin(ep(2.0));
        tx.object("a", f_root(), st(1.0), m())
            .object("b", f_root(), st(2.0), m());
        tx.commit();

        assert_eq!(obj_ids(&old), ["a"]); // previously-loaded snapshot unchanged
        let cur = store.snapshot();
        assert_eq!(obj_ids(&cur), ["a", "b"]);
    }

    #[test]
    fn object_metadata_round_trips() {
        use astrodyn_planet::MOON;
        let store = SceneStore::new();
        let mut tx = store.writer("sim").begin(ep(0.0));
        tx.object(
            "lander",
            f_moon(),
            st(1.0),
            ObjectMeta {
                label: "LM".into(),
                kind: ObjectKind::Spacecraft,
                shape: Some(BodyShape::ellipsoid(MOON)),
                path: Some(Path {
                    points: vec![DVec3::ZERO, DVec3::new(1.0, 0.0, 0.0)],
                }),
            },
        );
        tx.commit();

        let snap = store.snapshot();
        let lander = &snap.objects()[0];
        assert_eq!(lander.label, "LM");
        assert_eq!(lander.kind, ObjectKind::Spacecraft);
        assert_eq!(lander.shape.unwrap().ellipsoid.name, "Moon");
        assert_eq!(lander.path.as_ref().unwrap().points.len(), 2);
        // An object committed without metadata gets the defaults.
        let mut tx = store.writer("eph").begin(ep(0.0));
        tx.object("moon", f_root(), st(0.0), m());
        tx.commit();
        let snap = store.snapshot();
        let moon = snap
            .objects()
            .iter()
            .find(|o| o.id.as_str() == "moon")
            .unwrap();
        assert_eq!(moon.kind, ObjectKind::Marker);
        assert!(moon.label.is_empty() && moon.shape.is_none() && moon.path.is_none());
    }

    #[test]
    fn writer_is_send_and_concurrent_reads_stay_consistent() {
        let store = SceneStore::new();
        let mut tx = store.writer("sim").begin(ep(0.0));
        tx.object("a", f_root(), st(0.0), m());
        tx.commit();

        let writer = store.writer("sim"); // Send + Clone, moved to the producer thread
        let stop = Arc::new(AtomicBool::new(false));
        let stop_w = Arc::clone(&stop);
        let handle = thread::spawn(move || {
            let mut n = 0u64;
            while !stop_w.load(Ordering::Relaxed) {
                let mut tx = writer.begin(ep(n as f64));
                tx.object("a", f_root(), st(0.0), m());
                if n.is_multiple_of(2) {
                    tx.object("b", f_root(), st(1.0), m());
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
