//! Test-only helpers for golden-frame snapshot testing.
//!
//! Available in this crate's own tests and, for downstream crates, via the
//! `testing` feature (`astrotui-core = { ..., features = ["testing"] }` in
//! dev-dependencies). [`buffer_to_text`] renders a ratatui [`Buffer`] to a
//! deterministic grid of cell symbols suitable for `insta` snapshots.

use ratatui::buffer::Buffer;

/// Dump a [`Buffer`] to a deterministic string: one line per row, each cell's
/// symbol concatenated left to right. Stable across runs, so it snapshots cleanly.
pub fn buffer_to_text(buf: &Buffer) -> String {
    let area = *buf.area();
    let mut out = String::with_capacity((area.width as usize + 1) * area.height as usize);
    for y in area.top()..area.bottom() {
        for x in area.left()..area.right() {
            out.push_str(buf[(x, y)].symbol());
        }
        out.push('\n');
    }
    out
}
