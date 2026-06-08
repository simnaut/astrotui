//! astrotui's own object/scene wire layer, riding alongside `astrodyn_frame_doc` frames
//! (DESIGN.md §4.3). `astrodyn_frame_doc` models *frames only*; objects — which carry
//! `kind`/`shape`/`path` and name their frame by `FrameUid` — are astrotui's, so they get
//! their own records here and travel in a combined envelope:
//!
//! - [`SceneDocument`] = a frame [`FrameDocument`] snapshot + a `Vec<`[`ObjectRecord`]`>`.
//! - [`SceneSeries`] = a frame [`FrameSeries`] replay + a congruent object timeline.
//!
//! Both decode through any [`WireCodec`](crate::codec::WireCodec) (JSON now) and are applied to
//! a [`SceneWriter`] via [`apply_scene_document`] / [`apply_scene_series_epoch`], honoring the
//! keyframe handshake: **validate before interpreting any number** (DESIGN §4.3/§4.4), so an
//! orphan object, a non-finite field, a bad ellipsoid, or a misaligned timeline is surfaced
//! loudly and **nothing is committed**. The frame half reuses the consume helpers in
//! [`crate::frame_doc`]; objects add [`stage_objects`].

use std::borrow::Cow;
use std::collections::HashSet;
use std::sync::{Mutex, OnceLock};

use astrodyn_frame_doc::{
    CanonicalRotation, DocError, FrameDocument, FrameRecord, FrameSeries, TransRecord,
};
use astrodyn_planet::PlanetShape;
use astrodyn_quantities::FrameUid;
use astrotui_core::producer::Producer;
use astrotui_core::scene::{
    BodyShape, BodyState, ObjectKind, ObjectMeta, Path, SceneWriter, Transaction,
};
use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::frame_doc::{check_parents, rotation_to_dquat, stage_records, tx_epoch, ApplyError};

/// Object class on the wire. Mirrors core [`ObjectKind`] but is serde-owned by this crate
/// (core links no serde); the mapping is total and explicit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObjectKindWire {
    /// A celestial body.
    Body,
    /// A spacecraft / vehicle.
    Spacecraft,
    /// A fixed surface site.
    Site,
    /// A generic point of interest.
    Marker,
}

impl From<ObjectKindWire> for ObjectKind {
    fn from(k: ObjectKindWire) -> Self {
        match k {
            ObjectKindWire::Body => ObjectKind::Body,
            ObjectKindWire::Spacecraft => ObjectKind::Spacecraft,
            ObjectKindWire::Site => ObjectKind::Site,
            ObjectKindWire::Marker => ObjectKind::Marker,
        }
    }
}

impl From<ObjectKind> for ObjectKindWire {
    fn from(k: ObjectKind) -> Self {
        match k {
            ObjectKind::Body => ObjectKindWire::Body,
            ObjectKind::Spacecraft => ObjectKindWire::Spacecraft,
            ObjectKind::Site => ObjectKindWire::Site,
            ObjectKind::Marker => ObjectKindWire::Marker,
        }
    }
}

/// A reference-ellipsoid shape on the wire — the five [`PlanetShape`] parameters, flat
/// (the private radii carried explicitly; reconstructed via the validating `PlanetShape::new`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShapeRecord {
    /// Body name.
    pub name: String,
    /// Gravitational parameter (m³/s²).
    pub mu: f64,
    /// Mean equatorial radius (m).
    pub r_eq: f64,
    /// Mean polar radius (m).
    pub r_pol: f64,
    /// Flattening coefficient f = (r_eq − r_pol) / r_eq.
    pub flat_coeff: f64,
}

/// One scene object on the wire: id + label + frame + kind + state + optional shape/path.
///
/// Translation and attitude reuse `astrodyn_frame_doc`'s [`TransRecord`] / [`CanonicalRotation`]
/// so the wire is uniform and decode reuses [`rotation_to_dquat`]. The object names its frame by
/// **`frame_index`** into the embedding document's shared uid table (exactly as a
/// [`FrameRecord`]'s parent does), not by an inline `FrameUid` — one interned identity table,
/// and "object references a real frame" becomes the same kind of index check as the parent
/// guard.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectRecord {
    /// Stable object id (core `ObjectId`).
    pub id: String,
    /// Human label for the camera/frame switcher UI.
    pub label: String,
    /// Index of this object's frame in the document's uid table.
    pub frame_index: u32,
    /// What the object is.
    pub kind: ObjectKindWire,
    /// Position/velocity in the object's native frame (m, m/s).
    pub trans: TransRecord,
    /// Body attitude within its native frame (parent→this canonical rotation).
    pub rotation: CanonicalRotation,
    /// Optional reference-ellipsoid shape (drives LOD).
    pub shape: Option<ShapeRecord>,
    /// Optional planned polyline, native-frame metres.
    pub path: Option<Vec<[f64; 3]>>,
}

/// A scene **snapshot**: an `astrodyn_frame_doc` frame document plus astrotui's object layer,
/// both keyed off the document's `frames.uids` table.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SceneDocument {
    /// The frame half (astrodyn #659 wire).
    pub frames: FrameDocument,
    /// The object half (astrotui's).
    pub objects: Vec<ObjectRecord>,
}

/// One epoch of object state in a replay [`SceneSeries`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectEpochRow {
    /// Elapsed sim seconds for this row (must match the paired frame epoch's `simtime`).
    pub simtime: f64,
    /// Objects at this epoch.
    pub objects: Vec<ObjectRecord>,
}

/// A constant-topology run of object epochs, parallel to a frame `FrameSegment`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectSegment {
    /// Start sim seconds (must match the paired frame segment's `start_simtime`).
    pub start_simtime: f64,
    /// The object rows.
    pub epochs: Vec<ObjectEpochRow>,
}

/// A scene **replay**: an `astrodyn_frame_doc` [`FrameSeries`] plus a congruent object timeline.
/// astrodyn's `FrameSeries` can't be extended, so objects ride in a parallel
/// `Vec<`[`ObjectSegment`]`>` that [`SceneSeries::validate`] checks is **shape-congruent** with
/// `frames.segments` (same segment/epoch counts, bit-exact simtimes) so a player can apply frame
/// epoch `(s, e)` and object epoch `(s, e)` together.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SceneSeries {
    /// The frame timeline (astrodyn #659 wire).
    pub frames: FrameSeries,
    /// The object timeline, congruent to `frames.segments`.
    pub objects: Vec<ObjectSegment>,
}

/// A scene-level validation failure — astrotui's object/envelope checks, distinct from the
/// frame half's [`DocError`]. Surfaced through [`ApplyError::Scene`].
#[derive(Debug)]
pub enum SceneError {
    /// The frame half failed astrodyn validation (the keyframe handshake).
    Frames(DocError),
    /// An object names a frame that the document does not *place* (index out of range, or a uid
    /// with no `FrameRecord`) — the object→frame analogue of the dangling-parent guard.
    OrphanObject {
        /// The offending object's id.
        object: String,
        /// The frame index it named.
        frame_index: u32,
        /// The uid-table length.
        len: usize,
    },
    /// A numeric object field (trans / rotation / shape / path) is non-finite.
    NonFinite(String),
    /// A shape violates the ellipsoid invariants `PlanetShape::new` enforces — caught here so
    /// that constructor's panic is unreachable from a validated document.
    BadShape {
        /// The offending object's id.
        object: String,
        /// Which invariant was violated.
        why: &'static str,
    },
    /// The object timeline is not congruent with the frame timeline (segment/epoch counts or
    /// simtimes disagree) — a player could not align them, so it is rejected, never zipped short.
    SeriesMisaligned {
        /// Segment index involved.
        segment: usize,
        /// Epoch index, when the mismatch is within a segment's rows.
        epoch: Option<usize>,
        /// What disagreed.
        why: &'static str,
    },
}

impl From<DocError> for SceneError {
    fn from(e: DocError) -> Self {
        SceneError::Frames(e)
    }
}

impl std::fmt::Display for SceneError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SceneError::Frames(e) => write!(f, "frame half failed validation: {e}"),
            SceneError::OrphanObject {
                object,
                frame_index,
                len,
            } => write!(
                f,
                "object {object} names frame index {frame_index}, which is not a placed frame (uid table len {len})"
            ),
            SceneError::NonFinite(object) => {
                write!(f, "object {object} has a non-finite numeric field")
            }
            SceneError::BadShape { object, why } => {
                write!(f, "object {object} has an invalid shape: {why}")
            }
            SceneError::SeriesMisaligned {
                segment,
                epoch,
                why,
            } => match epoch {
                Some(e) => write!(
                    f,
                    "scene series misaligned at segment {segment}, epoch {e}: {why}"
                ),
                None => write!(f, "scene series misaligned at segment {segment}: {why}"),
            },
        }
    }
}

impl std::error::Error for SceneError {}

/// Intern a body name into a `&'static str` for [`PlanetShape::new`].
///
/// `PlanetShape::new` demands `&'static str` (its presets are `const`); a decoded [`ShapeRecord`]
/// only has a `String`. We leak each *distinct* name once into a process-global set and return
/// the stored `&'static str`. The leak is bounded by the number of distinct body names a process
/// ever decodes (Earth/Moon/… — a handful), **not** by message count: re-decoding `"Moon"` a
/// million times leaks nothing after the first. Hardening for untrusted streams of many distinct
/// names (an interner cap) is tracked in #86.
fn intern_body_name(name: &str) -> &'static str {
    static INTERN: OnceLock<Mutex<HashSet<&'static str>>> = OnceLock::new();
    let set = INTERN.get_or_init(|| Mutex::new(HashSet::new()));
    let mut guard = set
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    if let Some(s) = guard.get(name) {
        return s;
    }
    let leaked: &'static str = Box::leak(name.to_owned().into_boxed_str());
    guard.insert(leaked);
    leaked
}

/// Build a core [`BodyState`] from an object's wire translation + rotation (reuses the frame
/// half's [`rotation_to_dquat`], so objects and frames share one rotation convention).
fn object_body_state(trans: &TransRecord, rotation: &CanonicalRotation) -> BodyState {
    BodyState {
        position: DVec3::from_array(trans.position),
        velocity: DVec3::from_array(trans.velocity),
        attitude: rotation_to_dquat(rotation),
    }
}

/// Reconstruct a core [`BodyShape`] from a wire [`ShapeRecord`]. The ellipsoid invariants are
/// validated upstream (see [`SceneError::BadShape`]), so `PlanetShape::new` cannot panic here.
fn body_shape(s: &ShapeRecord) -> BodyShape {
    BodyShape {
        ellipsoid: PlanetShape::new(
            intern_body_name(&s.name),
            s.mu,
            s.r_eq,
            s.r_pol,
            s.flat_coeff,
        ),
    }
}

fn all_finite(xs: &[f64]) -> bool {
    xs.iter().all(|x| x.is_finite())
}

fn rotation_finite(r: &CanonicalRotation) -> bool {
    match r {
        CanonicalRotation::Quat(q) => all_finite(q),
        CanonicalRotation::Matrix(m) => m.iter().all(|row| all_finite(row)),
    }
}

/// The frames a document *places*: a `provided[i]` bitmap over the uid table, true where some
/// `FrameRecord` has `uid_index == i`. Mirrors the bitmap [`check_parents`] builds; call after
/// `frames.validate()` so `uid_index` is in range.
fn placed_frames(uids_len: usize, records: &[FrameRecord]) -> Vec<bool> {
    let mut placed = vec![false; uids_len];
    for r in records {
        if let Some(slot) = placed.get_mut(r.uid_index as usize) {
            *slot = true;
        }
    }
    placed
}

/// Validate one object against the placed-frame bitmap: it must name a placed frame, every
/// numeric field must be finite, and any shape must satisfy the ellipsoid invariants.
fn validate_object(uids_len: usize, placed: &[bool], o: &ObjectRecord) -> Result<(), SceneError> {
    let idx = o.frame_index as usize;
    if idx >= uids_len || !placed.get(idx).copied().unwrap_or(false) {
        return Err(SceneError::OrphanObject {
            object: o.id.clone(),
            frame_index: o.frame_index,
            len: uids_len,
        });
    }
    let path_ok = o
        .path
        .as_ref()
        .is_none_or(|p| p.iter().all(|pt| all_finite(pt)));
    if !all_finite(&o.trans.position)
        || !all_finite(&o.trans.velocity)
        || !rotation_finite(&o.rotation)
        || !path_ok
    {
        return Err(SceneError::NonFinite(o.id.clone()));
    }
    if let Some(s) = &o.shape {
        if !(s.mu.is_finite()
            && s.r_eq.is_finite()
            && s.r_pol.is_finite()
            && s.flat_coeff.is_finite())
        {
            return Err(SceneError::NonFinite(o.id.clone()));
        }
        // The invariants PlanetShape::new panics on — reject here so that panic is unreachable.
        if s.r_eq <= 0.0 {
            return Err(SceneError::BadShape {
                object: o.id.clone(),
                why: "r_eq must be > 0",
            });
        }
        if s.r_pol <= 0.0 {
            return Err(SceneError::BadShape {
                object: o.id.clone(),
                why: "r_pol must be > 0",
            });
        }
        if s.r_pol > s.r_eq {
            return Err(SceneError::BadShape {
                object: o.id.clone(),
                why: "r_pol must be <= r_eq (oblate or spherical only)",
            });
        }
    }
    Ok(())
}

fn validate_objects(
    uids_len: usize,
    placed: &[bool],
    objects: &[ObjectRecord],
) -> Result<(), SceneError> {
    for o in objects {
        validate_object(uids_len, placed, o)?;
    }
    Ok(())
}

impl SceneDocument {
    /// Validate the snapshot — the keyframe handshake for a scene. Runs the frame half's
    /// `validate()`, then for every object: placed-frame orphan check, finiteness, and shape
    /// invariants. A producer/decoder calls this before [`apply_scene_document`] (which
    /// re-validates regardless).
    ///
    /// # Errors
    /// [`SceneError`] on any frame or object violation; nothing is mutated.
    pub fn validate(&self) -> Result<(), SceneError> {
        self.frames.validate()?;
        let len = self.frames.uids.len();
        let placed = placed_frames(len, &self.frames.records);
        validate_objects(len, &placed, &self.objects)
    }
}

impl SceneSeries {
    /// Validate the replay — the frame half's `validate()`, the object timeline's **congruence**
    /// with the frame timeline (matching segment/epoch counts and bit-exact simtimes), and every
    /// object row against its epoch's placed frames.
    ///
    /// # Errors
    /// [`SceneError`] on any violation; nothing is mutated.
    pub fn validate(&self) -> Result<(), SceneError> {
        self.frames.validate()?;
        let len = self.frames.uids.len();
        if self.objects.len() != self.frames.segments.len() {
            return Err(SceneError::SeriesMisaligned {
                segment: 0,
                epoch: None,
                why: "segment count differs between the frame and object timelines",
            });
        }
        for (i, (fseg, oseg)) in self.frames.segments.iter().zip(&self.objects).enumerate() {
            if oseg.start_simtime.to_bits() != fseg.start_simtime.to_bits() {
                return Err(SceneError::SeriesMisaligned {
                    segment: i,
                    epoch: None,
                    why: "segment start_simtime differs",
                });
            }
            if oseg.epochs.len() != fseg.epochs.len() {
                return Err(SceneError::SeriesMisaligned {
                    segment: i,
                    epoch: None,
                    why: "epoch count differs",
                });
            }
            for (e, (frow, orow)) in fseg.epochs.iter().zip(&oseg.epochs).enumerate() {
                if orow.simtime.to_bits() != frow.simtime.to_bits() {
                    return Err(SceneError::SeriesMisaligned {
                        segment: i,
                        epoch: Some(e),
                        why: "epoch simtime differs",
                    });
                }
                let placed = placed_frames(len, &frow.records);
                validate_objects(len, &placed, &orow.objects)?;
            }
        }
        Ok(())
    }
}

/// Stage every object record onto an open transaction, resolving `frame_index` into a
/// [`FrameUid`]. Indices are guaranteed placed by the prior `validate()`; the `expect` ties any
/// violation back to that contract rather than panicking opaquely (matches [`stage_records`]).
pub(crate) fn stage_objects(tx: &mut Transaction, uids: &[FrameUid], objects: &[ObjectRecord]) {
    for o in objects {
        let frame = uids
            .get(o.frame_index as usize)
            .expect("frame_index placed (validated by apply_*)")
            .clone();
        let meta = ObjectMeta {
            label: Cow::Owned(o.label.clone()),
            kind: o.kind.into(),
            shape: o.shape.as_ref().map(body_shape),
            path: o.path.as_ref().map(|pts| Path {
                points: pts.iter().map(|p| DVec3::from_array(*p)).collect(),
            }),
        };
        tx.object(
            o.id.as_str(),
            frame,
            object_body_state(&o.trans, &o.rotation),
            meta,
        );
    }
}

/// Apply a [`SceneDocument`] snapshot onto `w` — frames + objects in one transaction.
///
/// Validates (frame handshake + object checks + the dangling-parent guard) **before** opening
/// the transaction, so a rejection commits nothing. Reuses the frame half's
/// [`stage_records`]/[`check_parents`]/[`tx_epoch`] and adds [`stage_objects`].
///
/// # Errors
/// [`ApplyError`] if the frame or object half is invalid, or a parent is dangling.
pub fn apply_scene_document(doc: &SceneDocument, w: &mut SceneWriter) -> Result<(), ApplyError> {
    doc.validate()?;
    check_parents(&doc.frames.uids, &doc.frames.records)?;
    let mut tx = w.begin(tx_epoch(&doc.frames.records));
    stage_records(&mut tx, &doc.frames.uids, &doc.frames.records);
    stage_objects(&mut tx, &doc.frames.uids, &doc.objects);
    tx.commit();
    Ok(())
}

/// Apply one epoch `(segment, epoch)` of a [`SceneSeries`] — the paired frame row and object
/// row together — onto `w`. The full time-driven replay player is #22.
///
/// # Errors
/// [`ApplyError::Scene`] if the series is invalid/misaligned, [`ApplyError::NoSuchEpoch`] if the
/// index is out of range, [`ApplyError::DanglingParent`] on a dangling parent.
pub fn apply_scene_series_epoch(
    series: &SceneSeries,
    segment: usize,
    epoch: usize,
    w: &mut SceneWriter,
) -> Result<(), ApplyError> {
    series.validate()?;
    let frow = series
        .frames
        .segments
        .get(segment)
        .and_then(|s| s.epochs.get(epoch))
        .ok_or(ApplyError::NoSuchEpoch)?;
    // validate() proved the timelines are congruent, so the object row exists too.
    let orow = series
        .objects
        .get(segment)
        .and_then(|s| s.epochs.get(epoch))
        .expect("object epoch present (congruent by validate())");
    check_parents(&series.frames.uids, &frow.records)?;
    let mut tx = w.begin(tx_epoch(&frow.records));
    stage_records(&mut tx, &series.frames.uids, &frow.records);
    stage_objects(&mut tx, &series.frames.uids, &orow.objects);
    tx.commit();
    Ok(())
}

/// A [`Producer`] that publishes a [`SceneDocument`] snapshot (frames + objects).
pub struct SceneDocumentProducer {
    /// The document to apply. [`apply_scene_document`] validates regardless of source.
    pub doc: SceneDocument,
}

impl SceneDocumentProducer {
    /// Apply the document, surfacing the handshake/validation outcome.
    ///
    /// # Errors
    /// [`ApplyError`] if the document fails validation.
    pub fn try_populate(&self, w: &mut SceneWriter) -> Result<(), ApplyError> {
        apply_scene_document(&self.doc, w)
    }
}

impl Producer for SceneDocumentProducer {
    /// Apply the document. A malformed in-process keyframe is a contract error, so this surfaces
    /// **loudly** (panics) rather than silently committing nothing — DESIGN §4.4. Callers wanting
    /// the error use [`SceneDocumentProducer::try_populate`].
    fn populate(&self, w: &mut SceneWriter) {
        self.try_populate(w)
            .expect("SceneDocumentProducer: scene document failed the keyframe handshake");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codec::{Json, WireCodec};
    use astrodyn_frame_doc::{Conventions, DocHeader, Origin, SeriesBuilder, SCHEMA_VERSION};
    use astrodyn_quantities::{Mars, Moon, PlanetFixed, RootInertial};
    use astrotui_core::scene::SceneStore;

    fn header() -> DocHeader {
        DocHeader {
            schema_version: SCHEMA_VERSION,
            conventions: Conventions::current(),
            simtime: 0.0,
            tai_tjt_at_epoch: 0.0,
        }
    }
    fn root_uid() -> FrameUid {
        FrameUid::of::<RootInertial>()
    }
    fn child_uid() -> FrameUid {
        FrameUid::of::<PlanetFixed<Moon>>()
    }
    fn ident() -> CanonicalRotation {
        CanonicalRotation::Quat([1.0, 0.0, 0.0, 0.0])
    }
    fn frec(
        name: &str,
        uid_index: u32,
        parent: Option<u32>,
        simtime: f64,
        pos: [f64; 3],
    ) -> FrameRecord {
        FrameRecord {
            name: name.into(),
            uid_index,
            parent,
            epoch: Some(simtime),
            trans: TransRecord {
                position: pos,
                velocity: [0.0; 3],
            },
            rotation: ident(),
            ang_vel_this: [0.0; 3],
            origin: Origin::Injected,
        }
    }
    // root + a placed child (PlanetFixed<Moon> at +x), the frame half objects ride on.
    fn base_frames() -> FrameDocument {
        FrameDocument {
            header: header(),
            uids: vec![root_uid(), child_uid()],
            records: vec![
                frec("root", 0, None, 0.0, [0.0; 3]),
                frec("moon_fixed", 1, Some(0), 0.0, [1.0, 2.0, 3.0]),
            ],
        }
    }
    fn obj(id: &str, frame_index: u32, pos: [f64; 3]) -> ObjectRecord {
        ObjectRecord {
            id: id.into(),
            label: format!("label-{id}"),
            frame_index,
            kind: ObjectKindWire::Spacecraft,
            trans: TransRecord {
                position: pos,
                velocity: [0.0; 3],
            },
            rotation: ident(),
            shape: None,
            path: None,
        }
    }
    fn earth_shape() -> ShapeRecord {
        ShapeRecord {
            name: "Earth".into(),
            mu: 3.986_004_418e14,
            r_eq: 6_378_137.0,
            r_pol: 6_356_752.314_245,
            flat_coeff: 0.003_352_810_664,
        }
    }

    #[test]
    fn json_round_trips_scene_document_via_codec() {
        let mut o = obj("lander", 1, [5.0, 0.0, 0.0]);
        o.shape = Some(earth_shape());
        o.path = Some(vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]);
        o.rotation = CanonicalRotation::Quat([0.0, 1.0, 0.0, 0.0]); // non-identity
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![o],
        };
        doc.validate().unwrap();
        let bytes = Json.encode(&doc).unwrap();
        let back: SceneDocument = Json.decode(&bytes).unwrap();
        back.validate().unwrap();
        assert_eq!(
            doc, back,
            "round-trip is identity (float_roundtrip fidelity)"
        );
    }

    #[test]
    fn apply_places_objects_in_frames() {
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![obj("lander", 1, [5.0, 0.0, 0.0])],
        };
        let store = SceneStore::new();
        apply_scene_document(&doc, &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        assert_eq!(snap.frames().len(), 2);
        let o = snap
            .objects()
            .iter()
            .find(|o| o.id.as_str() == "lander")
            .unwrap();
        assert_eq!(o.frame, child_uid());
        assert_eq!(o.kind, ObjectKind::Spacecraft);
        assert_eq!(o.label, "label-lander");
        assert_eq!(o.state.position, DVec3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn orphan_object_rejected_out_of_range() {
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![obj("ghost", 9, [0.0; 3])], // index past the uid table
        };
        let store = SceneStore::new();
        let err = apply_scene_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(
            err,
            ApplyError::Scene(SceneError::OrphanObject { .. })
        ));
        assert!(store.snapshot().is_empty(), "nothing committed on orphan");
    }

    #[test]
    fn orphan_object_rejected_unplaced_frame() {
        // Index 2 is a valid uid-table entry (Mars) but no FrameRecord places it.
        let mut frames = base_frames();
        frames.uids.push(FrameUid::of::<PlanetFixed<Mars>>());
        let doc = SceneDocument {
            frames,
            objects: vec![obj("on_unplaced", 2, [0.0; 3])],
        };
        let store = SceneStore::new();
        let err = apply_scene_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(
            err,
            ApplyError::Scene(SceneError::OrphanObject { .. })
        ));
        assert!(store.snapshot().is_empty());
    }

    #[test]
    fn shape_round_trips_including_name_interning() {
        let mut o = obj("earthish", 1, [0.0; 3]);
        o.shape = Some(earth_shape());
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![o],
        };
        // Apply twice into separate stores; the interner is process-global.
        let s1 = SceneStore::new();
        apply_scene_document(&doc, &mut s1.writer("wire")).unwrap();
        let s2 = SceneStore::new();
        apply_scene_document(&doc, &mut s2.writer("wire")).unwrap();
        let snap1 = s1.snapshot();
        let snap2 = s2.snapshot();
        let sh1 = snap1.objects()[0].shape.unwrap();
        let sh2 = snap2.objects()[0].shape.unwrap();
        // Values survive bit-exact.
        assert_eq!(sh1.ellipsoid.name, "Earth");
        assert_eq!(sh1.ellipsoid.mu, 3.986_004_418e14);
        assert_eq!(sh1.ellipsoid.r_eq(), 6_378_137.0);
        assert_eq!(sh1.ellipsoid.r_pol(), 6_356_752.314_245);
        // Same name interns to the SAME &'static str (no double leak).
        assert!(std::ptr::eq(
            sh1.ellipsoid.name.as_ptr(),
            sh2.ellipsoid.name.as_ptr()
        ));
    }

    #[test]
    fn bad_shape_rejected_before_planetshape_panic() {
        let mut o = obj("prolate", 1, [0.0; 3]);
        o.shape = Some(ShapeRecord {
            name: "Bad".into(),
            mu: 1.0,
            r_eq: 1.0,
            r_pol: 2.0, // prolate — PlanetShape::new would panic; we must reject first
            flat_coeff: 0.0,
        });
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![o],
        };
        let store = SceneStore::new();
        let err = apply_scene_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(
            err,
            ApplyError::Scene(SceneError::BadShape { .. })
        ));
        assert!(store.snapshot().is_empty());
    }

    #[test]
    fn non_finite_object_field_rejected() {
        let mut o = obj("nan", 1, [f64::NAN, 0.0, 0.0]);
        o.path = Some(vec![[0.0, 0.0, 0.0]]);
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![o],
        };
        let store = SceneStore::new();
        let err = apply_scene_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(err, ApplyError::Scene(SceneError::NonFinite(_))));
        assert!(store.snapshot().is_empty());
    }

    #[test]
    fn path_round_trips() {
        let mut o = obj("trail", 1, [0.0; 3]);
        o.path = Some(vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![o],
        };
        let store = SceneStore::new();
        apply_scene_document(&doc, &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        let path = snap.objects()[0].path.as_ref().unwrap();
        assert_eq!(
            path.points,
            vec![DVec3::new(1.0, 2.0, 3.0), DVec3::new(4.0, 5.0, 6.0)]
        );
    }

    // A two-epoch, single-segment series: child moves +x, an object moves with it.
    fn base_series() -> SceneSeries {
        let mut b = SeriesBuilder::new(header(), vec![root_uid(), child_uid()]);
        b.push_epoch(
            0.0,
            vec![
                frec("root", 0, None, 0.0, [0.0; 3]),
                frec("moon", 1, Some(0), 0.0, [10.0, 0.0, 0.0]),
            ],
        );
        b.push_epoch(
            1.0,
            vec![
                frec("root", 0, None, 1.0, [0.0; 3]),
                frec("moon", 1, Some(0), 1.0, [20.0, 0.0, 0.0]),
            ],
        );
        let frames = b.finish();
        let objects = vec![ObjectSegment {
            start_simtime: 0.0,
            epochs: vec![
                ObjectEpochRow {
                    simtime: 0.0,
                    objects: vec![obj("probe", 1, [1.0, 0.0, 0.0])],
                },
                ObjectEpochRow {
                    simtime: 1.0,
                    objects: vec![obj("probe", 1, [2.0, 0.0, 0.0])],
                },
            ],
        }];
        SceneSeries { frames, objects }
    }

    #[test]
    fn series_per_epoch_apply() {
        let series = base_series();
        let store = SceneStore::new();
        apply_scene_series_epoch(&series, 0, 1, &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        let probe = snap
            .objects()
            .iter()
            .find(|o| o.id.as_str() == "probe")
            .unwrap();
        assert_eq!(probe.state.position, DVec3::new(2.0, 0.0, 0.0)); // epoch index 1
        let child = snap.frames().iter().find(|f| f.uid == child_uid()).unwrap();
        assert_eq!(child.state.position, DVec3::new(20.0, 0.0, 0.0));

        // Out-of-range epoch is reported, not panicked.
        let err = apply_scene_series_epoch(&series, 0, 9, &mut SceneStore::new().writer("wire"))
            .unwrap_err();
        assert!(matches!(err, ApplyError::NoSuchEpoch));
    }

    #[test]
    fn series_misalignment_rejected() {
        // (a) epoch-count mismatch: drop one object epoch.
        let mut s = base_series();
        s.objects[0].epochs.pop();
        let err =
            apply_scene_series_epoch(&s, 0, 0, &mut SceneStore::new().writer("wire")).unwrap_err();
        assert!(matches!(
            err,
            ApplyError::Scene(SceneError::SeriesMisaligned { .. })
        ));

        // (b) simtime bit-mismatch at epoch 1.
        let mut s = base_series();
        s.objects[0].epochs[1].simtime = 1.5;
        let store = SceneStore::new();
        let err = apply_scene_series_epoch(&s, 0, 1, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(
            err,
            ApplyError::Scene(SceneError::SeriesMisaligned { epoch: Some(1), .. })
        ));
        assert!(store.snapshot().is_empty());

        // (c) segment-count mismatch.
        let mut s = base_series();
        s.objects.clear();
        let err =
            apply_scene_series_epoch(&s, 0, 0, &mut SceneStore::new().writer("wire")).unwrap_err();
        assert!(matches!(
            err,
            ApplyError::Scene(SceneError::SeriesMisaligned { .. })
        ));
    }

    #[test]
    fn series_json_round_trips_via_codec() {
        let series = base_series();
        series.validate().unwrap();
        let bytes = Json.encode(&series).unwrap();
        let back: SceneSeries = Json.decode(&bytes).unwrap();
        assert_eq!(series, back);
    }

    #[test]
    fn scene_document_producer_populates() {
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![obj("lander", 1, [0.0; 3])],
        };
        let store = SceneStore::new();
        SceneDocumentProducer { doc }.populate(&mut store.writer("wire"));
        let snap = store.snapshot();
        assert_eq!(snap.frames().len(), 2);
        assert_eq!(snap.objects().len(), 1);
    }

    #[test]
    #[should_panic(expected = "keyframe handshake")]
    fn scene_document_producer_panics_on_malformed() {
        let doc = SceneDocument {
            frames: base_frames(),
            objects: vec![obj("ghost", 9, [0.0; 3])], // orphan
        };
        SceneDocumentProducer { doc }.populate(&mut SceneStore::new().writer("wire"));
    }
}
