//! astrotui-core — the source-agnostic rendering core.
//!
//! Holds the widget, [`Camera`](crate), the `SceneStore`/`SceneWriter` ingestion
//! model, the `Renderer` trait, and the projection/LOD pipeline. Per `docs/DESIGN.md`
//! (§2, §12) this crate links **no Bevy** and **no ANISE/ephemeris** — it renders the
//! states it is given. Implementation lands in P0 and later.
