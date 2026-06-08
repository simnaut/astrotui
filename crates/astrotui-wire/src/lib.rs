//! astrotui-wire — the scene wire adapter.
//!
//! Consumes astrodyn's `astrodyn_frame_doc` schema (astrodyn #659) — `FrameDocument`
//! snapshots and `FrameSeries` replay — into the core `SceneStore` via the
//! [`Producer`](astrotui_core::producer::Producer) seam, honoring the keyframe handshake
//! (the header is validated before any state is interpreted; see `docs/DESIGN.md` §4.3).
//! The frame wire is `astrodyn_frame_doc`; this crate adds the **outer envelope** + the
//! **object/scene layer** that rides alongside it (frame_doc is frames-only) in [`scene_doc`],
//! behind an encoding-agnostic [`codec::WireCodec`] (JSON now; binary is P3). This is the only
//! crate that depends on `astrodyn_frame_doc` and writes serde derives — `astrotui-core` holds
//! `FrameUid` without serde (DESIGN §3); the serde stack lives here.

pub mod codec;
pub mod frame_doc;
pub mod scene_doc;

pub use codec::{Json, WireCodec};
pub use frame_doc::{apply_document, apply_series_epoch, ApplyError, DocumentProducer};
pub use scene_doc::{
    apply_scene_document, apply_scene_series_epoch, ObjectEpochRow, ObjectKindWire, ObjectRecord,
    ObjectSegment, SceneDocument, SceneDocumentProducer, SceneError, SceneSeries, ShapeRecord,
};
