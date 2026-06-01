//! orchestrator — the reference astrotui host.
//!
//! Long-lived TUI app that owns the `SceneStore` + camera and drives producers
//! (spawned sim over the wire, replay files, live telemetry). The real wiring lands in
//! P0 (#16, first demo) and P3 (lifecycle, telemetry). This binary is currently a
//! placeholder so the workspace builds end-to-end.

fn main() {
    // P0 (#16) wires the first demo: an in-process producer feeding Earth/Moon/Sun
    // points into a SceneStore rendered by the braille backend.
}
