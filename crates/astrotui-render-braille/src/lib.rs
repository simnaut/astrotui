//! astrotui-render-braille — monochrome braille-dot backend (the capability floor).
//!
//! Rasterizes projected points into a ratatui `Buffer` using Unicode braille
//! (U+2800–U+28FF): each terminal cell is a 2×4 grid of dots, giving points ~2× horizontal
//! and ~4× vertical resolution over plain cells. Implements
//! [`astrotui_core::render::Renderer`].

use astrotui_core::render::{RenderBody, RenderKind, Renderer};
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;

/// Braille cell height:width — terminal cells are ~2× taller than wide. Ellipse rows are divided
/// by this so a silhouette that is round in the col metric renders round on screen.
const CELL_ASPECT: f64 = 2.0;

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
    fn draw_bodies(&self, bodies: &[RenderBody], area: Rect, buf: &mut Buffer) {
        let (w, h) = (area.width as usize, area.height as usize);
        if w == 0 || h == 0 {
            return;
        }
        // One accumulator byte per terminal cell, OR-ing in a bit per lit 2×4 sub-cell dot.
        let mut dots = vec![0u8; w * h];
        for b in bodies {
            match b.kind {
                RenderKind::Point => plot_dot(&mut dots, w, h, b.col, b.row),
                RenderKind::Ellipsoid {
                    semi_major,
                    semi_minor,
                    tilt,
                } => fill_ellipse(
                    &mut dots,
                    w,
                    h,
                    (b.col, b.row),
                    semi_major,
                    semi_minor,
                    tilt,
                ),
            }
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

/// OR one sub-cell dot at fractional `(col, row)` (local to the area) into the accumulator.
/// Non-finite / negative / out-of-grid coordinates are ignored (never panic).
fn plot_dot(dots: &mut [u8], w: usize, h: usize, col: f64, row: f64) {
    let dx = col * 2.0;
    let dy = row * 4.0;
    if !dx.is_finite() || !dy.is_finite() || dx < 0.0 || dy < 0.0 {
        return;
    }
    let (dx, dy) = (dx as usize, dy as usize);
    if dx >= w * 2 || dy >= h * 4 {
        return;
    }
    dots[(dy / 4) * w + dx / 2] |= braille_bit(dx % 2, dy % 4);
}

/// Rasterize a filled, oriented ellipse silhouette centred at `(cx, cy)` into the accumulator.
/// Semi-axes `(a, b)` are in **col** cells (`a` = major, along `tilt`; `b` = minor); rows are
/// aspect-corrected by [`CELL_ASPECT`] so a round silhouette renders round. Sampled at braille
/// sub-cell resolution (½ cell wide, ¼ cell tall). A circle (`a == b`, `tilt == 0`) reduces to a
/// plain filled disc.
fn fill_ellipse(
    dots: &mut [u8],
    w: usize,
    h: usize,
    center: (f64, f64),
    a: f64,
    b: f64,
    tilt: f64,
) {
    let (cx, cy) = center;
    if !(a > 0.0
        && b > 0.0
        && a.is_finite()
        && b.is_finite()
        && cx.is_finite()
        && cy.is_finite()
        && tilt.is_finite())
    {
        return;
    }
    let (sin_t, cos_t) = tilt.sin_cos();
    let rcol = a.max(b); // bounding half-extent in col cells
    let rrow = rcol / CELL_ASPECT;
    // Clamp the sample box to the viewport: samples outside are dropped by `plot_dot` anyway, so
    // this bounds the work to the visible area (a huge ellipse doesn't iterate millions of
    // off-screen sub-cells — and would never terminate if a semi-axis were unbounded).
    let col_hi = (cx + rcol).min(w as f64);
    let row_hi = (cy + rrow).min(h as f64);
    let mut col = (cx - rcol).max(0.0);
    while col <= col_hi {
        let mut row = (cy - rrow).max(0.0);
        while row <= row_hi {
            let dx = col - cx;
            // Back to the square col-metric space (down +), then rotate the sample into the
            // ellipse's own axes and normalize by each semi-axis.
            let dy = (row - cy) * CELL_ASPECT;
            let u = (dx * cos_t + dy * sin_t) / a;
            let v = (-dx * sin_t + dy * cos_t) / b;
            if u * u + v * v <= 1.0 {
                plot_dot(dots, w, h, col, row);
            }
            row += 0.25;
        }
        col += 0.5;
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

    // ---- #19: ellipse silhouette ----

    fn render_bodies(bodies: &[RenderBody], area: Rect) -> Buffer {
        let mut buf = Buffer::empty(area);
        BrailleRenderer::new().draw_bodies(bodies, area, &mut buf);
        buf
    }
    fn lit(buf: &Buffer, area: Rect) -> Vec<(u16, u16)> {
        let mut v = Vec::new();
        for y in 0..area.height {
            for x in 0..area.width {
                if buf[(area.x + x, area.y + y)].symbol() != " " {
                    v.push((x, y));
                }
            }
        }
        v
    }

    #[test]
    fn fill_ellipse_circle_is_aspect_corrected() {
        // A circle (semi_major == semi_minor = 4 cells) at the centre of a 20×10 area. Because
        // braille cells are 2:1, a round silhouette spans ~2·r cols but ~r rows (≈ 2r/CELL_ASPECT).
        let a = Rect::new(0, 0, 20, 10);
        let body = RenderBody {
            col: 10.0,
            row: 5.0,
            kind: RenderKind::Ellipsoid {
                semi_major: 4.0,
                semi_minor: 4.0,
                tilt: 0.0,
            },
        };
        let cells = lit(&render_bodies(&[body], a), a);
        let span = |it: Vec<u16>| *it.iter().max().unwrap() - *it.iter().min().unwrap();
        let col_span = span(
            cells
                .iter()
                .filter(|(_, y)| *y == 5)
                .map(|(x, _)| *x)
                .collect(),
        );
        let row_span = span(
            cells
                .iter()
                .filter(|(x, _)| *x == 10)
                .map(|(_, y)| *y)
                .collect(),
        );
        assert!((7..=9).contains(&col_span), "col span {col_span} ≈ 2·4");
        assert!(
            (3..=5).contains(&row_span),
            "row span {row_span} ≈ 2·4/CELL_ASPECT"
        );
    }

    #[test]
    fn ellipsoid_fills_more_than_a_point() {
        let a = Rect::new(0, 0, 20, 10);
        let centre = (10.0, 5.0);
        let point = RenderBody {
            col: centre.0,
            row: centre.1,
            kind: RenderKind::Point,
        };
        let disc = RenderBody {
            col: centre.0,
            row: centre.1,
            kind: RenderKind::Ellipsoid {
                semi_major: 3.0,
                semi_minor: 3.0,
                tilt: 0.0,
            },
        };
        assert_eq!(lit(&render_bodies(&[point], a), a).len(), 1); // a point lights one cell
        assert!(lit(&render_bodies(&[disc], a), a).len() > 10); // a disc lights many
    }

    #[test]
    fn fill_ellipse_huge_and_nonfinite_are_bounded() {
        let a = Rect::new(0, 0, 20, 10);
        // A huge ellipse fills the visible area without iterating its (enormous) full bbox.
        let huge = RenderBody {
            col: 10.0,
            row: 5.0,
            kind: RenderKind::Ellipsoid {
                semi_major: 1.0e6,
                semi_minor: 1.0e6,
                tilt: 0.0,
            },
        };
        assert!(lit(&render_bodies(&[huge], a), a).len() > 100);
        // A non-finite semi-axis is rejected — no infinite loop, nothing drawn.
        let inf = RenderBody {
            col: 10.0,
            row: 5.0,
            kind: RenderKind::Ellipsoid {
                semi_major: f64::INFINITY,
                semi_minor: 4.0,
                tilt: 0.0,
            },
        };
        assert!(lit(&render_bodies(&[inf], a), a).is_empty());
    }

    #[test]
    fn oblate_ellipse_silhouette_golden() {
        // A tilted oblate ellipse (major 5, minor 2.5, tilt 30°) — a golden frame locks the
        // oriented, aspect-corrected rasterization (CLAUDE.md: prefer insta + buffer_to_text).
        let a = Rect::new(0, 0, 18, 9);
        let body = RenderBody {
            col: 9.0,
            row: 4.5,
            kind: RenderKind::Ellipsoid {
                semi_major: 5.0,
                semi_minor: 2.5,
                tilt: std::f64::consts::FRAC_PI_6,
            },
        };
        let buf = render_bodies(&[body], a);
        insta::assert_snapshot!(astrotui_core::testing::buffer_to_text(&buf));
    }
}
