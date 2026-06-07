//! astrotui-wire — the scene wire adapter.
//!
//! Consumes astrodyn's `astrodyn_frame_doc` schema (astrodyn #659) — `FrameDocument`
//! snapshots and `FrameSeries` replay — into the core `SceneStore` via the
//! [`Producer`](astrotui_core::producer::Producer) seam, honoring the keyframe handshake
//! (the header is validated before any state is interpreted; see `docs/DESIGN.md` §4.3).
//! The frame wire is `astrodyn_frame_doc`; this crate adds the outer framing and (later) the
//! object/scene layer that rides alongside (frame_doc is frames-only). This is the only
//! crate that links `serde` / `astrodyn_frame_doc`.

pub mod frame_doc;

pub use frame_doc::{apply_document, apply_series_epoch, ApplyError, DocumentProducer};
