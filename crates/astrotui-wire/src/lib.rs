//! astrotui-wire — the self-describing scene wire codec.
//!
//! One codec for the socket stream (live sim / telemetry) and replay files: a scene
//! header (FrameTree topology + object metadata) followed by frame-tagged samples
//! (see `docs/DESIGN.md` §4.3). Codec lands in P1 (#21); JSON first, binary framing in
//! P3 (#36). This is the only crate that enables `serde`.
