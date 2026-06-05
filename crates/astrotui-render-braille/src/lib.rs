//! astrotui-render-braille — monochrome braille-dot backend (the capability floor).
//!
//! Rasterizes projected points into a ratatui `Buffer` using Unicode braille
//! (U+2800–U+28FF): each terminal cell is a 2×4 grid of dots, giving points ~2× horizontal
//! and ~4× vertical resolution over plain cells. Implements
//! [`astrotui_core::render::Renderer`].

use astrotui_core::render::Renderer;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;

/// The braille point renderer — the `Renderer` capability floor (tui-globe style).
#[derive(Clone, Copy, Debug, Default)]
pub struct BrailleRenderer;

impl BrailleRenderer {
    /// Create a braille renderer.
    #[must_use]
    pub fn new() -> Self {
        Self
    }
}

impl Renderer for BrailleRenderer {
    fn draw_points(&self, points: &[(f64, f64)], area: Rect, buf: &mut Buffer) {
        let (w, h) = (area.width as usize, area.height as usize);
        if w == 0 || h == 0 {
            return;
        }
        // One accumulator byte per terminal cell, OR-ing in a bit per lit 2×4 sub-cell dot.
        let mut dots = vec![0u8; w * h];
        for &(col, row) in points {
            // Points are local to `area` ((0, 0) = top-left); the area offset is added when
            // writing cells below.
            let dx = col * 2.0;
            let dy = row * 4.0;
            if !dx.is_finite() || !dy.is_finite() || dx < 0.0 || dy < 0.0 {
                continue;
            }
            let (dx, dy) = (dx as usize, dy as usize);
            if dx >= w * 2 || dy >= h * 4 {
                continue;
            }
            dots[(dy / 4) * w + dx / 2] |= braille_bit(dx % 2, dy % 4);
        }
        for cy in 0..h {
            for cx in 0..w {
                let bits = dots[cy * w + cx];
                if bits != 0 {
                    // 0x2800 + bits (bits ∈ 0..=255) is always a valid braille code point.
                    let ch = char::from_u32(0x2800 + u32::from(bits)).unwrap_or(' ');
                    buf[(area.x + cx as u16, area.y + cy as u16)].set_char(ch);
                }
            }
        }
    }
}

/// The braille dot bit for sub-cell position `(x, y)`, `x ∈ 0..2`, `y ∈ 0..4`, using the
/// standard 2×4 dot numbering (dots 1–3 / 4–6 down the columns, 7–8 across the bottom).
fn braille_bit(x: usize, y: usize) -> u8 {
    match (x, y) {
        (0, 0) => 0x01,
        (0, 1) => 0x02,
        (0, 2) => 0x04,
        (0, 3) => 0x40,
        (1, 0) => 0x08,
        (1, 1) => 0x10,
        (1, 2) => 0x20,
        (1, 3) => 0x80,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn render(points: &[(f64, f64)], area: Rect) -> Buffer {
        let mut buf = Buffer::empty(area);
        BrailleRenderer::new().draw_points(points, area, &mut buf);
        buf
    }
    fn glyph(buf: &Buffer, x: u16, y: u16) -> String {
        buf[(x, y)].symbol().to_string()
    }

    #[test]
    fn plots_subcell_dots() {
        let a = Rect::new(0, 0, 1, 1);
        assert_eq!(glyph(&render(&[(0.0, 0.0)], a), 0, 0), "\u{2801}"); // top-left, dot 1
        assert_eq!(glyph(&render(&[(0.5, 0.0)], a), 0, 0), "\u{2808}"); // top-right, dot 4
        assert_eq!(glyph(&render(&[(0.5, 0.75)], a), 0, 0), "\u{2880}"); // bottom-right, dot 8
    }

    #[test]
    fn dots_in_one_cell_combine() {
        let a = Rect::new(0, 0, 1, 1);
        // dot 1 (0x01) | dot 4 (0x08) = 0x09 → U+2809.
        assert_eq!(
            glyph(&render(&[(0.0, 0.0), (0.5, 0.0)], a), 0, 0),
            "\u{2809}"
        );
    }

    #[test]
    fn point_lands_in_the_correct_cell_others_blank() {
        let a = Rect::new(0, 0, 4, 2);
        let buf = render(&[(2.0, 0.0)], a); // dx=4 → cell col 2; dy=0 → cell row 0, dot 1
        assert_eq!(glyph(&buf, 2, 0), "\u{2801}");
        assert_eq!(glyph(&buf, 0, 0), " "); // untouched cells stay blank
        assert_eq!(glyph(&buf, 1, 1), " ");
    }

    #[test]
    fn out_of_bounds_and_degenerate_inputs_are_ignored() {
        let a = Rect::new(0, 0, 1, 1);
        // Off the 1×1 cell, negative, and non-finite — none panic, none drawn.
        let buf = render(&[(1.0, 0.0), (-0.1, 0.0), (f64::NAN, 0.0)], a);
        assert_eq!(glyph(&buf, 0, 0), " ");
        // Zero-area viewport is a no-op.
        BrailleRenderer::new().draw_points(
            &[(0.0, 0.0)],
            Rect::new(0, 0, 0, 0),
            &mut Buffer::empty(a),
        );
    }

    #[test]
    fn renders_into_an_offset_area() {
        // Local (0,0) maps to the area's top-left cell (2,1): the area offset is applied
        // when writing, not only for (0,0)-origin areas.
        let a = Rect::new(2, 1, 3, 2);
        let buf = render(&[(0.0, 0.0)], a);
        assert_eq!(glyph(&buf, 2, 1), "\u{2801}");
        assert_eq!(glyph(&buf, 4, 2), " "); // another in-area cell, untouched
    }
}
