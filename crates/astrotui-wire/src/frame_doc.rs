//! Consume astrodyn's `astrodyn_frame_doc` wire schema into a scene (DESIGN.md §4.3).
//!
//! astrodyn #659 ships a physics-free, serde-based frame-document schema; astrotui consumes
//! its record types directly. This module turns a [`FrameDocument`] (snapshot) or one epoch
//! of a [`FrameSeries`] (replay) into [`SceneWriter`] frame commits, honoring the **keyframe
//! handshake**: the header (schema version + conventions) and structure are validated
//! *before* any state is interpreted.
//!
//! **Frames only.** `astrodyn_frame_doc` models no objects; astrotui's object/scene layer is
//! separate and rides alongside, referencing frames by `FrameUid`. The full stepping/seek
//! replay player is a later task (#22-roadmap); the per-record parent self-check + loud
//! surfacing is #76.

use astrodyn_frame_doc::{CanonicalRotation, DocError, FrameDocument, FrameRecord, FrameSeries};
use astrodyn_quantities::FrameUid;
use astrotui_core::producer::Producer;
use astrotui_core::scene::{BodyState, Epoch, SceneWriter, Transaction};
use glam::{DMat3, DQuat, DVec3};

/// Failure applying a wire document/series to a scene.
#[derive(Debug)]
pub enum ApplyError {
    /// The header (schema version / conventions) or structure failed validation — the
    /// keyframe handshake rejected the document before any state was applied.
    Invalid(DocError),
    /// A record names a `parent` uid that no record in the set provides — a dangling parent
    /// (the RFS-301/302 transplant guard against a stale-parent ~10⁵ km failure). astrodyn's
    /// `validate` only checks the parent *index* is in range, not that it refers to a record.
    DanglingParent {
        /// The record whose parent is missing.
        child: FrameUid,
        /// The named-but-absent parent.
        parent: FrameUid,
    },
    /// The requested segment/epoch index is out of range for the series.
    NoSuchEpoch,
    /// A scene-level validation failure (orphan object, non-finite object field, bad shape,
    /// or a frame/object timeline mismatch) — see [`crate::scene_doc::SceneError`] (#21).
    Scene(crate::scene_doc::SceneError),
}

impl From<crate::scene_doc::SceneError> for ApplyError {
    fn from(e: crate::scene_doc::SceneError) -> Self {
        ApplyError::Scene(e)
    }
}

impl From<DocError> for ApplyError {
    fn from(e: DocError) -> Self {
        ApplyError::Invalid(e)
    }
}

impl std::fmt::Display for ApplyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ApplyError::Invalid(e) => write!(f, "invalid frame document (keyframe handshake): {e}"),
            ApplyError::DanglingParent { child, parent } => {
                write!(
                    f,
                    "frame {child} names parent {parent}, which has no record"
                )
            }
            ApplyError::NoSuchEpoch => write!(f, "series segment/epoch index out of range"),
            ApplyError::Scene(e) => write!(f, "scene wire error: {e}"),
        }
    }
}

impl std::error::Error for ApplyError {}

/// Convert a canonical rotation (parent → this) to a glam quaternion.
///
/// `astrodyn_frame_doc` stores a quaternion scalar-first `[q0, q1, q2, q3]`; glam is
/// scalar-last `(x, y, z, w)`. A matrix is column-major (parent → this). This is a
/// **structural** conversion only — the exact JEOD left-transform ↔ glam convention is
/// validated by the rotating-frame integration test (#77); the renderer does not read
/// attitude yet, so #75 just carries the value faithfully.
pub(crate) fn rotation_to_dquat(r: &CanonicalRotation) -> DQuat {
    match r {
        CanonicalRotation::Quat([q0, q1, q2, q3]) => DQuat::from_xyzw(*q1, *q2, *q3, *q0),
        CanonicalRotation::Matrix(cols) => DQuat::from_mat3(&DMat3::from_cols(
            DVec3::from_array(cols[0]),
            DVec3::from_array(cols[1]),
            DVec3::from_array(cols[2]),
        )),
    }
}

/// Build a core [`BodyState`] from a wire record. Position/velocity are parent-frame SI;
/// attitude is the record's canonical rotation.
///
/// `ang_vel_this` and the [`astrodyn_frame_doc::Origin`] payload are intentionally **not**
/// carried: `BodyState` has no angular-velocity field and the renderer consumes none today.
/// Revisit when a consumer needs them (likely with the P2 rotating-frame work).
fn body_state(rec: &FrameRecord) -> BodyState {
    BodyState {
        position: DVec3::from_array(rec.trans.position),
        velocity: DVec3::from_array(rec.trans.velocity),
        attitude: rotation_to_dquat(&rec.rotation),
    }
}

/// The transaction epoch for a set of records: the first record's epoch (TDB seconds) if
/// present, else zero. A snapshot is a single instant, so one layer epoch is correct; the
/// `SceneWriter` stamps every staged frame with the transaction epoch, so per-record epoch
/// granularity is collapsed here (matches the current ingestion API). The header `simtime`
/// is deliberately NOT used — it is elapsed sim seconds, a different scale than
/// `Epoch = SecondsSince<TDB>`.
pub(crate) fn tx_epoch(records: &[FrameRecord]) -> Epoch {
    let secs = records.first().and_then(|r| r.epoch).unwrap_or(0.0);
    Epoch::from_seconds(secs)
}

/// Stage every record onto an open transaction, resolving uid-table indices into
/// [`FrameUid`]s. Index ranges are guaranteed in-bounds by the prior `validate()`; the
/// `expect` ties any violation back to that contract rather than panicking opaquely.
pub(crate) fn stage_records(tx: &mut Transaction, uids: &[FrameUid], records: &[FrameRecord]) {
    for rec in records {
        let uid = uids
            .get(rec.uid_index as usize)
            .expect("uid_index in range (validated by apply_*)")
            .clone();
        let parent = rec.parent.map(|p| {
            uids.get(p as usize)
                .expect("parent index in range (validated by apply_*)")
                .clone()
        });
        tx.frame(uid, parent, body_state(rec));
    }
}

/// Apply a [`FrameDocument`] snapshot onto `w` — the keyframe handshake.
///
/// Validates the header (schema version + conventions) and structure **before** interpreting
/// any state (DESIGN §4.3), then commits all frame records as one transaction. Returns
/// without committing anything if validation fails. Safe for both `from_json_str`-decoded and
/// literal-constructed documents (validation is the single choke point).
pub fn apply_document(doc: &FrameDocument, w: &mut SceneWriter) -> Result<(), ApplyError> {
    doc.validate()?;
    check_parents(&doc.uids, &doc.records)?;
    let mut tx = w.begin(tx_epoch(&doc.records));
    stage_records(&mut tx, &doc.uids, &doc.records);
    tx.commit();
    Ok(())
}

/// Verify every record's `parent` uid is provided by some record in the set (the per-record
/// parent self-check against the folded topology). `astrodyn_frame_doc::validate` checks a
/// parent *index* is in range but not that it names an existing record, so a dangling parent
/// slips past it — caught loudly here. Call after `validate()` (indices are assumed in range).
pub(crate) fn check_parents(uids: &[FrameUid], records: &[FrameRecord]) -> Result<(), ApplyError> {
    // Use `get(...).expect("…validated by apply_*")` (as `stage_records` does) so a violated
    // upstream contract surfaces a clear message, not an opaque bounds panic.
    let uid = |i: u32| -> FrameUid {
        uids.get(i as usize)
            .expect("uid index in range (validated by apply_*)")
            .clone()
    };
    let mut provided = vec![false; uids.len()];
    for r in records {
        *provided
            .get_mut(r.uid_index as usize)
            .expect("uid_index in range (validated by apply_*)") = true;
    }
    for r in records {
        if let Some(p) = r.parent {
            let parent_provided = *provided
                .get(p as usize)
                .expect("parent index in range (validated by apply_*)");
            if !parent_provided {
                return Err(ApplyError::DanglingParent {
                    child: uid(r.uid_index),
                    parent: uid(p),
                });
            }
        }
    }
    Ok(())
}

/// Apply one epoch row of a [`FrameSeries`] (replay, single row) onto `w`. The full
/// stepping/seek replay player is a separate task (#22-roadmap).
pub fn apply_series_epoch(
    series: &FrameSeries,
    segment: usize,
    epoch: usize,
    w: &mut SceneWriter,
) -> Result<(), ApplyError> {
    series.validate()?;
    let row = series
        .segments
        .get(segment)
        .and_then(|s| s.epochs.get(epoch))
        .ok_or(ApplyError::NoSuchEpoch)?;
    check_parents(&series.uids, &row.records)?;
    let mut tx = w.begin(tx_epoch(&row.records));
    stage_records(&mut tx, &series.uids, &row.records);
    tx.commit();
    Ok(())
}

/// A [`Producer`] that publishes a [`FrameDocument`] snapshot's frames.
pub struct DocumentProducer {
    /// The document to apply. Typically from `FrameDocument::from_json_str` (which validates);
    /// [`apply_document`] re-validates regardless.
    pub doc: FrameDocument,
}

impl DocumentProducer {
    /// Apply the document, surfacing the handshake/validation outcome.
    pub fn try_populate(&self, w: &mut SceneWriter) -> Result<(), ApplyError> {
        apply_document(&self.doc, w)
    }
}

impl Producer for DocumentProducer {
    /// Apply the document. A malformed in-process keyframe is a contract error at this stage,
    /// so this surfaces **loudly** (panics) rather than silently committing nothing — DESIGN
    /// §4.4 (loud, never silent). The streaming reader (P3) will handle stream-level errors.
    /// Callers wanting the error use [`DocumentProducer::try_populate`].
    fn populate(&self, w: &mut SceneWriter) {
        self.try_populate(w)
            .expect("DocumentProducer: frame document failed the keyframe handshake");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use astrodyn_frame_doc::{
        Conventions, DocHeader, Origin, SeriesBuilder, TransRecord, SCHEMA_VERSION,
    };
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

    fn rec(
        name: &str,
        uid_index: u32,
        parent: Option<u32>,
        epoch: f64,
        pos: [f64; 3],
        rot: CanonicalRotation,
    ) -> FrameRecord {
        FrameRecord {
            name: name.into(),
            uid_index,
            parent,
            epoch: Some(epoch),
            trans: TransRecord {
                position: pos,
                velocity: [0.0; 3],
            },
            rotation: rot,
            ang_vel_this: [0.0; 3],
            origin: Origin::Injected,
        }
    }

    fn ident() -> CanonicalRotation {
        CanonicalRotation::Quat([1.0, 0.0, 0.0, 0.0])
    }

    // root (RootInertial) + a child (PlanetFixed<Moon>) at +x with a non-identity rotation.
    fn two_frame_doc() -> FrameDocument {
        FrameDocument {
            header: header(),
            uids: vec![root_uid(), child_uid()],
            records: vec![
                rec("root", 0, None, 100.0, [0.0; 3], ident()),
                // scalar-first [0,1,0,0] -> glam (x=1,y=0,z=0,w=0)
                rec(
                    "moon_fixed",
                    1,
                    Some(0),
                    100.0,
                    [1.0, 2.0, 3.0],
                    CanonicalRotation::Quat([0.0, 1.0, 0.0, 0.0]),
                ),
            ],
        }
    }

    #[test]
    fn applies_root_and_rotating_child() {
        let store = SceneStore::new();
        apply_document(&two_frame_doc(), &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        assert_eq!(snap.frames().len(), 2);

        let child = snap
            .frames()
            .iter()
            .find(|f| f.uid == child_uid())
            .expect("child frame present");
        assert_eq!(child.parent, Some(root_uid()));
        assert_eq!(child.state.position, DVec3::new(1.0, 2.0, 3.0));
        // scalar-first [0,1,0,0] -> xyzw (1,0,0,0)
        assert!(child
            .state
            .attitude
            .abs_diff_eq(DQuat::from_xyzw(1.0, 0.0, 0.0, 0.0), 1e-12));
    }

    #[test]
    fn places_frames_not_objects() {
        let store = SceneStore::new();
        apply_document(&two_frame_doc(), &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        assert!(snap.objects().is_empty(), "frame_doc carries no objects");
        assert!(!snap.frames().is_empty());
    }

    #[test]
    fn matrix_rotation_converts() {
        // Round-trip a known quaternion through its column-major matrix and back, proving the
        // column ordering of the Matrix conversion (allowing the quaternion double-cover).
        let original = DQuat::from_rotation_z(0.7);
        let m = DMat3::from_quat(original);
        let cols = [
            m.x_axis.to_array(),
            m.y_axis.to_array(),
            m.z_axis.to_array(),
        ];
        let got = rotation_to_dquat(&CanonicalRotation::Matrix(cols));
        assert!(
            got.abs_diff_eq(original, 1e-12) || got.abs_diff_eq(-original, 1e-12),
            "matrix->quat mismatch: {got:?} vs {original:?}"
        );
    }

    #[test]
    fn json_round_trip_then_apply() {
        let json = two_frame_doc().to_json_string();
        let back = FrameDocument::from_json_str(&json).expect("valid json");
        let store = SceneStore::new();
        apply_document(&back, &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        assert_eq!(snap.frames().len(), 2);
        let child = snap.frames().iter().find(|f| f.uid == child_uid()).unwrap();
        assert_eq!(child.state.position, DVec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn handshake_rejects_bad_schema() {
        let mut doc = two_frame_doc();
        doc.header.schema_version = 999;
        let store = SceneStore::new();
        let err = apply_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(err, ApplyError::Invalid(_)));
        assert!(
            store.snapshot().is_empty(),
            "nothing committed on bad header"
        );
    }

    #[test]
    fn handshake_rejects_bad_conventions() {
        let mut doc = two_frame_doc();
        doc.header.conventions.time_scale = "wrong scale".into();
        let store = SceneStore::new();
        let err = apply_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(err, ApplyError::Invalid(_)));
        assert!(store.snapshot().is_empty());
    }

    #[test]
    fn rejects_dangling_parent() {
        // A record names a parent uid that no record provides. astrodyn `validate` passes
        // (the parent index is in range), but the per-record self-check rejects it loudly and
        // commits nothing — the RFS-301/302 stale-parent guard.
        let doc = FrameDocument {
            header: header(),
            uids: vec![root_uid(), child_uid(), FrameUid::of::<PlanetFixed<Mars>>()],
            records: vec![
                rec("root", 0, None, 0.0, [0.0; 3], ident()),
                // parent index 2 = Mars uid, but no record has uid_index 2 → dangling.
                rec("moon", 1, Some(2), 0.0, [0.0; 3], ident()),
            ],
        };
        doc.validate()
            .expect("astrodyn validate passes: parent index is in range");
        let store = SceneStore::new();
        let err = apply_document(&doc, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(err, ApplyError::DanglingParent { .. }));
        assert!(
            store.snapshot().is_empty(),
            "nothing committed on dangling parent"
        );
    }

    #[test]
    fn epoch_from_first_record() {
        let store = SceneStore::new();
        apply_document(&two_frame_doc(), &mut store.writer("wire")).unwrap();
        let secs = store
            .snapshot()
            .epoch(&"wire".into())
            .map(|e| e.as_seconds());
        assert_eq!(secs, Some(100.0));
    }

    #[test]
    fn apply_series_epoch_applies_one_row() {
        let mut b = SeriesBuilder::new(header(), vec![root_uid(), child_uid()]);
        let row = |t: f64, x: f64| -> Vec<FrameRecord> {
            vec![
                rec("root", 0, None, t, [0.0; 3], ident()),
                rec("moon_fixed", 1, Some(0), t, [x, 0.0, 0.0], ident()),
            ]
        };
        b.push_epoch(0.0, row(0.0, 1.0));
        b.push_epoch(1.0, row(1.0, 2.0));
        let series = b.finish();

        let store = SceneStore::new();
        apply_series_epoch(&series, 0, 1, &mut store.writer("wire")).unwrap();
        let snap = store.snapshot();
        let child = snap.frames().iter().find(|f| f.uid == child_uid()).unwrap();
        assert_eq!(child.state.position, DVec3::new(2.0, 0.0, 0.0)); // epoch index 1

        // Out-of-range epoch is reported, not panicked.
        let err =
            apply_series_epoch(&series, 0, 9, &mut SceneStore::new().writer("wire")).unwrap_err();
        assert!(matches!(err, ApplyError::NoSuchEpoch));
    }

    // #77 — the end-to-end rotating-frame proof that gates P2. A `CanonicalRotation` on a
    // frame must survive the full pipeline (doc quat → glam `DQuat` → `BodyState.attitude` →
    // `frame_state` → JEOD `q_parent_this` → `FrameTree` → `compute_relative_state` → camera
    // state) and orient both an object's *position* and its *body axes* (`att_cam`). Asserted
    // against hand-typed references so a convention slip (active/passive, scalar position,
    // transpose direction) fails loudly rather than silently mis-orienting the Moon in P2.
    #[test]
    fn rotating_frame_orients_position_and_attitude_through_camera() {
        use astrotui_core::render::{project_points, Camera};
        use astrotui_core::scene::ObjectMeta;
        use ratatui::layout::Rect;
        use std::f64::consts::FRAC_1_SQRT_2;

        // Child frame (PlanetFixed<Moon>) carrying a 90°-about-Z rotation relative to root,
        // offset by `offset`. Doc quat is scalar-first glam convention [w, x, y, z], so
        // [c, 0, 0, s] decodes to glam `DQuat::from_rotation_z(+π/2)`. The values below pin
        // the *observed* end-to-end convention: through `JeodQuat::from_glam` →
        // `left_quat_to_transformation` → `compute_relative_state`, this orients the child so
        // its +x basis axis points along root **−y** (i.e. child→root maps x→−y, y→+x). A
        // future convention slip flips these signs and fails the asserts below.
        let offset = [10.0, 20.0, 30.0];
        let z90 = || CanonicalRotation::Quat([FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2]);
        let doc = FrameDocument {
            header: header(),
            uids: vec![root_uid(), child_uid()],
            records: vec![
                rec("root", 0, None, 0.0, [0.0; 3], ident()),
                rec("moon_fixed", 1, Some(0), 0.0, offset, z90()),
            ],
        };

        // Exercise the real consume path: serialize and decode (the deserialize the issue
        // calls for) before applying, so the rotation survives the wire, not just a literal.
        let doc = FrameDocument::from_json_str(&doc.to_json_string()).expect("valid json");

        let store = SceneStore::new();
        apply_document(&doc, &mut store.writer("wire")).unwrap();
        // frame_doc carries no objects; place two in the rotated child via a separate layer.
        // `probe` has identity attitude (isolates the frame rotation); `tilted` carries its
        // own +90°-about-Z body attitude (exercises the body→frame→camera composition).
        let d = 5.0;
        let writer = store.writer("obj");
        let mut tx = writer.begin(Epoch::from_seconds(0.0));
        tx.object(
            "probe",
            child_uid(),
            BodyState {
                position: DVec3::new(d, 0.0, 0.0),
                velocity: DVec3::ZERO,
                attitude: DQuat::IDENTITY,
            },
            ObjectMeta::default(),
        )
        .object(
            "tilted",
            child_uid(),
            BodyState {
                position: DVec3::ZERO,
                velocity: DVec3::ZERO,
                attitude: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
            },
            ObjectMeta::default(),
        );
        tx.commit();

        let snap = store.snapshot();
        let area = Rect::new(0, 0, 400, 400);
        let (pts, report) = project_points(&snap, &Camera::overview(root_uid(), 1.0), area);
        assert!(
            report.is_clean(),
            "rotating-frame scene must resolve cleanly: {report:?}"
        );

        let probe = pts.iter().find(|p| p.id.as_str() == "probe").unwrap();
        // The child's +x axis points along root −y, so the probe at child [5,0,0] maps to
        // root [0,−5,0], plus the frame offset → [10,15,30].
        let expected_pos = DVec3::new(offset[0], offset[1] - d, offset[2]);
        assert!(
            probe.pos_cam.abs_diff_eq(expected_pos, 1e-9),
            "probe pos_cam {:?} != {expected_pos:?}",
            probe.pos_cam
        );
        // Identity-attitude object: att_cam == frame→camera (child→root). Its columns are the
        // child basis axes in root coords: child-x→−y, child-y→+x, child-z→+z.
        let child_to_root = DMat3::from_cols(
            DVec3::new(0.0, -1.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 0.0, 1.0),
        );
        assert!(
            probe.att_cam.abs_diff_eq(child_to_root, 1e-9),
            "probe att_cam {:?} != {child_to_root:?}",
            probe.att_cam
        );

        // The tilted object adds another +90° about Z on top of the frame's +90° → 180°
        // about Z in camera coords (body axes: +x→−x, +y→−y, +z→+z).
        let tilted = pts.iter().find(|p| p.id.as_str() == "tilted").unwrap();
        let body_to_root = DMat3::from_cols(
            DVec3::new(-1.0, 0.0, 0.0),
            DVec3::new(0.0, -1.0, 0.0),
            DVec3::new(0.0, 0.0, 1.0),
        );
        assert!(
            tilted.att_cam.abs_diff_eq(body_to_root, 1e-9),
            "tilted att_cam {:?} != {body_to_root:?}",
            tilted.att_cam
        );

        // The loud parent-mismatch path, with a rotating frame in play: a rotated child whose
        // named parent has no record must still be rejected (nothing committed), proving the
        // RFS-301/302 stale-parent guard isn't bypassed once rotations are present.
        let dangling = FrameDocument {
            header: header(),
            uids: vec![root_uid(), child_uid(), FrameUid::of::<PlanetFixed<Mars>>()],
            records: vec![
                rec("root", 0, None, 0.0, [0.0; 3], ident()),
                // parent index 2 (Mars) is a valid index but has no record → dangling.
                rec("moon_fixed", 1, Some(2), 0.0, offset, z90()),
            ],
        };
        let store = SceneStore::new();
        let err = apply_document(&dangling, &mut store.writer("wire")).unwrap_err();
        assert!(matches!(err, ApplyError::DanglingParent { .. }));
        assert!(
            store.snapshot().is_empty(),
            "nothing committed on dangling parent"
        );
    }

    #[test]
    fn document_producer_populates() {
        let store = SceneStore::new();
        DocumentProducer {
            doc: two_frame_doc(),
        }
        .populate(&mut store.writer("wire"));
        assert_eq!(store.snapshot().frames().len(), 2);
    }
}
