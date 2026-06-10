//! astrotui-render-color — 24-bit color half-block backend with dithered shading.
//!
//! Rasterizes projected bodies into a ratatui `Buffer` using half-block cells: each
//! terminal cell is two vertically stacked "pixels" (`▀` with independent fg/bg RGB),
//! giving ~square pixels at 1×2 the cell resolution. Ellipsoids are shaded as
//! limb-darkened spheres through a pluggable [`Shader`] — the hypsometric/hillshade
//! hook the DEM stages (#29) extend. Implements [`astrotui_core::render::Renderer`].

use astrotui_core::render::{RenderBody, RenderKind, Renderer};
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::Color;

/// Cell height:width — terminal cells are ~2× taller than wide. With two half-block
/// pixels per cell, pixels come out ~square, so a silhouette that is round in the col
/// metric renders round on screen.
const CELL_ASPECT: f64 = 2.0;

/// What a [`Shader`] sees for one covered pixel of an ellipsoid silhouette.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShadeSample {
    /// Limb-darkened sphere intensity in `[0, 1]`: the z of the unit sphere at this
    /// silhouette point (1 at the centre, → 0 at the limb), i.e. Lambert with the
    /// light along the view ray.
    pub lambert: f64,
    /// Silhouette-space coordinate along the major (equatorial) axis, in `[-1, 1]`.
    pub u: f64,
    /// Silhouette-space coordinate along the minor (polar) axis, in `[-1, 1]`.
    pub v: f64,
}

/// Maps a covered pixel to linear RGB in `[0, 1]` per channel. This is the
/// hypsometric/hillshade hook: the default [`LambertGray`] shades a neutral sphere;
/// DEM Stage 4 (#29) supplies shaders keyed on height/normal instead. Quantization
/// (with ordered dithering) is the renderer's job, not the shader's.
pub trait Shader {
    /// Linear RGB for one covered pixel.
    fn shade(&self, sample: ShadeSample) -> [f64; 3];
}

/// The default shader: a neutral limb-darkened sphere (white × lambert).
#[derive(Clone, Copy, Debug, Default)]
pub struct LambertGray;

impl Shader for LambertGray {
    fn shade(&self, sample: ShadeSample) -> [f64; 3] {
        [sample.lambert; 3]
    }
}

/// The 24-bit half-block renderer (DESIGN.md §5.1: color cells).
#[derive(Clone, Copy, Debug, Default)]
pub struct ColorRenderer<S = LambertGray> {
    shader: S,
}

impl ColorRenderer<LambertGray> {
    /// A color renderer with the default neutral-sphere shader.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
}

impl<S: Shader> ColorRenderer<S> {
    /// A color renderer with a custom [`Shader`] (the hypsometric/hillshade hook).
    pub fn with_shader(shader: S) -> Self {
        Self { shader }
    }
}

impl<S: Shader> Renderer for ColorRenderer<S> {
    fn draw_bodies(&self, bodies: &[RenderBody], area: Rect, buf: &mut Buffer) {
        let (w, h) = (area.width as usize, area.height as usize);
        if w == 0 || h == 0 {
            return;
        }
        // One optional RGB per half-block pixel, w × 2h; later bodies overwrite earlier
        // ones per pixel (the projection pass hands bodies in painter's order).
        let mut px = vec![None::<[u8; 3]>; w * h * 2];
        for b in bodies {
            match b.kind {
                RenderKind::Point => self.plot_point(&mut px, w, h, b.col, b.row),
                RenderKind::Ellipsoid {
                    semi_major,
                    semi_minor,
                    tilt,
                } => self.fill_ellipse(
                    &mut px,
                    w,
                    h,
                    (b.col, b.row),
                    (semi_major, semi_minor),
                    tilt,
                ),
            }
        }
        for cy in 0..h {
            for cx in 0..w {
                let top = px[(cy * 2) * w + cx];
                let bottom = px[(cy * 2 + 1) * w + cx];
                let cell = &mut buf[(area.x + cx as u16, area.y + cy as u16)];
                match (top, bottom) {
                    (None, None) => {}
                    // Only one half covered: pick the half-block glyph whose foreground
                    // is the covered half, leaving the cell's background untouched.
                    (Some(t), None) => {
                        cell.set_char('▀').set_fg(rgb(t));
                    }
                    (None, Some(b)) => {
                        cell.set_char('▄').set_fg(rgb(b));
                    }
                    (Some(t), Some(b)) => {
                        cell.set_char('▀').set_fg(rgb(t)).set_bg(rgb(b));
                    }
                }
            }
        }
    }
}

impl<S: Shader> ColorRenderer<S> {
    /// Set the single half-block pixel under fractional `(col, row)` (local to the
    /// area) at full intensity. Non-finite / negative / out-of-grid coordinates are
    /// ignored (never panic).
    fn plot_point(&self, px: &mut [Option<[u8; 3]>], w: usize, h: usize, col: f64, row: f64) {
        let y = row * 2.0;
        if !col.is_finite() || !y.is_finite() || col < 0.0 || y < 0.0 {
            return;
        }
        let (x, y) = (col as usize, y as usize);
        if x >= w || y >= h * 2 {
            return;
        }
        let sample = ShadeSample {
            lambert: 1.0,
            u: 0.0,
            v: 0.0,
        };
        px[y * w + x] = Some(quantize(self.shader.shade(sample), x, y));
    }

    /// Rasterize a filled, oriented, shaded ellipse silhouette centred at `(cx, cy)`.
    /// `semi` is `(a, b)` in **col** cells (`a` = major, along `tilt`; `b` = minor);
    /// rows are aspect-corrected by [`CELL_ASPECT`]. Sampled at half-block pixel
    /// centres; the pixel ranges are clamped to the viewport so a huge ellipse never
    /// iterates off-screen pixels.
    fn fill_ellipse(
        &self,
        px: &mut [Option<[u8; 3]>],
        w: usize,
        h: usize,
        center: (f64, f64),
        semi: (f64, f64),
        tilt: f64,
    ) {
        let (cx, cy) = center;
        let (a, b) = semi;
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
        let rcol = a.max(b); // bounding half-extent, col cells
        let rrow = rcol / CELL_ASPECT; // … in rows
        let x_lo = ((cx - rcol).floor().max(0.0)) as usize;
        let x_hi = ((cx + rcol).ceil().min(w as f64)).max(0.0) as usize;
        let y_lo = (((cy - rrow) * 2.0).floor().max(0.0)) as usize;
        let y_hi = (((cy + rrow) * 2.0).ceil().min(h as f64 * 2.0)).max(0.0) as usize;
        for y in y_lo..y_hi {
            for x in x_lo..x_hi {
                // Pixel centre back in (col, row) cell coordinates, then into the
                // square col-metric space (down +), the ellipse's own axes, and
                // normalized by each semi-axis — as the braille backend does.
                let dx = (x as f64 + 0.5) - cx;
                let dy = ((y as f64 + 0.5) / 2.0 - cy) * CELL_ASPECT;
                let u = (dx * cos_t + dy * sin_t) / a;
                let v = (-dx * sin_t + dy * cos_t) / b;
                let rho2 = u * u + v * v;
                if rho2 <= 1.0 {
                    let sample = ShadeSample {
                        lambert: (1.0 - rho2).sqrt(),
                        u,
                        v,
                    };
                    px[y * w + x] = Some(quantize(self.shader.shade(sample), x, y));
                }
            }
        }
    }
}

/// 4×4 ordered (Bayer) dither thresholds. Indexed `[y % 4][x % 4]`; values 0–15.
const BAYER_4X4: [[u8; 4]; 4] = [[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]];

/// Quantize linear `[0, 1]` RGB to 8-bit with ordered dithering: the Bayer threshold
/// for this pixel position decides whether the fractional remainder rounds up, so
/// smooth gradients break into a stable dot pattern instead of banding (visible when
/// the terminal downsamples truecolor). 0.0 → 0 and 1.0 → 255 exactly.
fn quantize(rgb: [f64; 3], x: usize, y: usize) -> [u8; 3] {
    let t = (f64::from(BAYER_4X4[y % 4][x % 4]) + 0.5) / 16.0;
    rgb.map(|c| {
        let scaled = c.clamp(0.0, 1.0) * 255.0;
        let base = scaled.floor();
        let up = (scaled - base) > t;
        (base as u8).saturating_add(u8::from(up))
    })
}

/// ratatui truecolor from quantized RGB.
fn rgb([r, g, b]: [u8; 3]) -> Color {
    Color::Rgb(r, g, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn render_bodies(bodies: &[RenderBody], area: Rect) -> Buffer {
        let mut buf = Buffer::empty(area);
        ColorRenderer::new().draw_bodies(bodies, area, &mut buf);
        buf
    }

    fn ellipsoid(col: f64, row: f64, a: f64, b: f64, tilt: f64) -> RenderBody {
        RenderBody {
            col,
            row,
            kind: RenderKind::Ellipsoid {
                semi_major: a,
                semi_minor: b,
                tilt,
            },
        }
    }

    /// The two half-block pixels of a cell as `(top, bottom)` RGB, decoded from the
    /// glyph + fg/bg the renderer wrote. `None` = that half untouched.
    fn cell_pixels(buf: &Buffer, x: u16, y: u16) -> (Option<[u8; 3]>, Option<[u8; 3]>) {
        let cell = &buf[(x, y)];
        let chan = |c: Color| match c {
            Color::Rgb(r, g, b) => Some([r, g, b]),
            _ => None,
        };
        match cell.symbol() {
            "▀" => (chan(cell.fg), chan(cell.bg)),
            "▄" => (None, chan(cell.fg)),
            _ => (None, None),
        }
    }

    /// Luminance ramp dump for golden frames: two text rows per cell row (top/bottom
    /// half-block pixels), darkest→brightest ` .:-=+*#%@`.
    fn pixels_to_text(buf: &Buffer, area: Rect) -> String {
        const RAMP: &[u8] = b" .:-=+*#%@";
        let mut out = String::new();
        for y in 0..area.height {
            let mut rows = [String::new(), String::new()];
            for x in 0..area.width {
                let (top, bottom) = cell_pixels(buf, area.x + x, area.y + y);
                for (row, p) in rows.iter_mut().zip([top, bottom]) {
                    let ch = match p {
                        None => ' ',
                        Some([r, g, b]) => {
                            let lum = (u32::from(r) + u32::from(g) + u32::from(b)) / 3;
                            RAMP[(lum as usize * (RAMP.len() - 1)) / 255] as char
                        }
                    };
                    row.push(ch);
                }
            }
            for row in rows {
                out.push_str(&row);
                out.push('\n');
            }
        }
        out
    }

    #[test]
    fn point_sets_one_half_block() {
        let a = Rect::new(0, 0, 4, 2);
        // row 0.0 → top half of cell row 0.
        let buf = render_bodies(
            &[RenderBody {
                col: 2.0,
                row: 0.0,
                kind: RenderKind::Point,
            }],
            a,
        );
        assert_eq!(buf[(2, 0)].symbol(), "▀");
        assert_eq!(buf[(2, 0)].fg, Color::Rgb(255, 255, 255));
        assert_eq!(buf[(0, 0)].symbol(), " "); // untouched cells stay blank

        // row 0.5 → bottom half: the lower-half glyph, background untouched.
        let buf = render_bodies(
            &[RenderBody {
                col: 1.0,
                row: 0.5,
                kind: RenderKind::Point,
            }],
            a,
        );
        assert_eq!(buf[(1, 0)].symbol(), "▄");
        assert_eq!(buf[(1, 0)].fg, Color::Rgb(255, 255, 255));
    }

    #[test]
    fn both_halves_use_fg_and_bg() {
        let a = Rect::new(0, 0, 3, 3);
        // A disc covering several full cells: its centre cell has both halves set.
        let buf = render_bodies(&[ellipsoid(1.5, 1.5, 1.5, 1.5, 0.0)], a);
        let (top, bottom) = cell_pixels(&buf, 1, 1);
        assert_eq!(buf[(1, 1)].symbol(), "▀");
        assert!(top.is_some() && bottom.is_some());
        assert!(matches!(buf[(1, 1)].bg, Color::Rgb(..)));
    }

    #[test]
    fn circle_is_aspect_corrected() {
        // A circle (r = 4 col cells) spans ~2r cols but ~r rows (2:1 cells).
        let a = Rect::new(0, 0, 20, 10);
        let buf = render_bodies(&[ellipsoid(10.0, 5.0, 4.0, 4.0, 0.0)], a);
        let lit: Vec<(u16, u16)> = (0..10)
            .flat_map(|y| (0..20).map(move |x| (x, y)))
            .filter(|&(x, y)| buf[(x, y)].symbol() != " ")
            .collect();
        let span = |it: Vec<u16>| *it.iter().max().unwrap() - *it.iter().min().unwrap();
        let col_span = span(lit.iter().filter(|(_, y)| *y == 5).map(|p| p.0).collect());
        let row_span = span(lit.iter().filter(|(x, _)| *x == 10).map(|p| p.1).collect());
        assert!((7..=9).contains(&col_span), "col span {col_span} ≈ 2·4");
        assert!((3..=5).contains(&row_span), "row span {row_span} ≈ 4");
    }

    #[test]
    fn shading_darkens_toward_the_limb() {
        let a = Rect::new(0, 0, 20, 10);
        let buf = render_bodies(&[ellipsoid(10.0, 5.0, 6.0, 6.0, 0.0)], a);
        let lum = |x: u16, y: u16| {
            let (top, _) = cell_pixels(&buf, x, y);
            top.map(|[r, ..]| r).unwrap_or(0)
        };
        // Centre is near full intensity; a cell near the limb is markedly darker.
        assert!(lum(10, 5) > 220, "centre {}", lum(10, 5));
        assert!(
            lum(5, 5) < lum(10, 5) - 60,
            "limb {} vs centre {}",
            lum(5, 5),
            lum(10, 5)
        );
    }

    #[test]
    fn dither_breaks_flat_midtones() {
        // A constant intensity between two 8-bit codes must quantize to ≥ 2 distinct
        // levels across a 4×4 Bayer block; the exact extremes stay exact.
        let mid = 127.5 / 255.0;
        let vals: Vec<u8> = (0..4)
            .flat_map(|y| (0..4).map(move |x| quantize([mid; 3], x, y)[0]))
            .collect();
        assert!(vals.iter().any(|&v| v != vals[0]), "no dither: {vals:?}");
        assert_eq!(quantize([0.0; 3], 1, 2), [0, 0, 0]);
        assert_eq!(quantize([1.0; 3], 3, 1), [255, 255, 255]);
    }

    #[test]
    fn out_of_bounds_and_degenerate_inputs_are_ignored() {
        let a = Rect::new(0, 0, 1, 1);
        let pts = [(1.0, 0.0), (-0.1, 0.0), (f64::NAN, 0.0)];
        let mut buf = Buffer::empty(a);
        ColorRenderer::new().draw_points(&pts, a, &mut buf);
        assert_eq!(buf[(0, 0)].symbol(), " ");
        // Zero-area viewport is a no-op; non-finite semi-axis draws nothing.
        ColorRenderer::new().draw_points(&[(0.0, 0.0)], Rect::new(0, 0, 0, 0), &mut buf);
        let buf = render_bodies(&[ellipsoid(0.5, 0.5, f64::INFINITY, 1.0, 0.0)], a);
        assert_eq!(buf[(0, 0)].symbol(), " ");
    }

    #[test]
    fn huge_ellipse_is_bounded() {
        let a = Rect::new(0, 0, 20, 10);
        let buf = render_bodies(&[ellipsoid(10.0, 5.0, 1.0e6, 1.0e6, 0.0)], a);
        assert!(buf[(0, 0)].symbol() != " " && buf[(19, 9)].symbol() != " ");
    }

    #[test]
    fn renders_into_an_offset_area() {
        // Local (0,0) maps to the area's top-left cell: the offset applies on write.
        let a = Rect::new(2, 1, 3, 2);
        let mut buf = Buffer::empty(a);
        ColorRenderer::new().draw_points(&[(0.0, 0.0)], a, &mut buf);
        assert_eq!(buf[(2, 1)].symbol(), "▀");
        assert_eq!(buf[(4, 2)].symbol(), " ");
    }

    #[test]
    fn custom_shader_hook_is_used() {
        // The hypsometric hook: a shader keyed on the polar coordinate v.
        struct PolarTint;
        impl Shader for PolarTint {
            fn shade(&self, s: ShadeSample) -> [f64; 3] {
                [s.lambert, (s.v + 1.0) / 2.0, 0.0]
            }
        }
        let a = Rect::new(0, 0, 12, 6);
        let mut buf = Buffer::empty(a);
        ColorRenderer::with_shader(PolarTint).draw_bodies(
            &[ellipsoid(6.0, 3.0, 4.0, 4.0, 0.0)],
            a,
            &mut buf,
        );
        // Green grows toward +v (down): a southern cell is greener than a northern one.
        let g = |x: u16, y: u16| match cell_pixels(&buf, x, y).0 {
            Some([_, g, _]) => g,
            None => 0,
        };
        assert!(g(6, 4) > g(6, 1), "south {} vs north {}", g(6, 4), g(6, 1));
    }

    #[test]
    fn shaded_ellipse_golden() {
        // A tilted oblate ellipse, same geometry as the braille golden, but shaded —
        // the luminance dump locks both the silhouette and the limb-darkening ramp.
        let a = Rect::new(0, 0, 18, 9);
        let buf = render_bodies(
            &[ellipsoid(9.0, 4.5, 5.0, 2.5, std::f64::consts::FRAC_PI_6)],
            a,
        );
        insta::assert_snapshot!(pixels_to_text(&buf, a));
    }
}
