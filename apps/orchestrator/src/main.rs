//! orchestrator — the reference astrotui host.
//!
//! Demo (#16, extended): a trivial in-process producer places the Sun, Earth, and Moon on
//! a `RootInertial` frame and the app draws them through the camera = frame → project →
//! rasterize pipeline. Bodies large enough on screen are drawn as filled braille discs from
//! their `BodyShape` radius; smaller ones collapse to a point.
//!
//! This is a **stylized, not-to-scale diagram** — at true scale Earth can't fill a fifth of
//! the screen while the Sun (1 AU away, 109× Earth's radius) and Moon (60 Earth-radii away)
//! stay in frame; that ~12-orders-of-magnitude span is what the P1 seamless log-zoom solves.
//! The disc fill here previews the P1 angular-size LOD (point → shaded ellipsoid, #19).
//! Press `q` or `Esc` to quit.

use std::io;
use std::time::Duration;

use astrodyn_planet::{PlanetShape, EARTH, MOON, SUN};
use astrotui_core::render::Renderer;
use astrotui_core::scene::{BodyShape, BodyState, Epoch, ObjectKind, ObjectMeta, SceneStore};
use astrotui_render_braille::BrailleRenderer;
use glam::DVec3;
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::Rect;

/// Terminal cells are roughly twice as tall as wide; correct for it so discs look round.
const CELL_ASPECT: f64 = 2.0;
/// Earth is drawn this fraction of the viewport width (> 0.2, per the brief).
const EARTH_SCREEN_FRACTION: f64 = 0.25;

/// Earth's equatorial radius (m) — the unit the stylized layout is expressed in.
fn earth_r() -> f64 {
    EARTH.r_eq()
}

/// Build the stylized Sun–Earth–Moon scene on the root inertial frame. Distances are
/// compressed (Earth-radius units) so all three stay in one frame; the Sun's radius is
/// stylized down (it is really 109× Earth) so it reads as a body rather than the whole sky.
fn build_scene() -> SceneStore {
    let store = SceneStore::new();
    let sun_shape = PlanetShape::new("Sun", SUN.mu, 2.2 * earth_r(), 2.2 * earth_r(), 0.0);
    let mut tx = store.writer("demo").begin(Epoch::from_seconds(0.0));
    tx.frame("root", None, BodyState::default());
    tx.object(
        "sun",
        "root",
        point(-4.6 * earth_r()),
        body("Sun", sun_shape),
    )
    .object("earth", "root", point(0.0), body("Earth", EARTH))
    .object("moon", "root", point(3.2 * earth_r()), body("Moon", MOON));
    tx.commit();
    store
}

fn point(x: f64) -> BodyState {
    BodyState {
        position: DVec3::new(x, 0.0, 0.0),
        ..BodyState::default()
    }
}

fn body(label: &'static str, shape: PlanetShape) -> ObjectMeta {
    ObjectMeta {
        label: label.into(),
        kind: ObjectKind::Body,
        shape: Some(BodyShape::ellipsoid(shape)),
        ..ObjectMeta::default()
    }
}

/// Metres per cell that makes Earth span [`EARTH_SCREEN_FRACTION`] of the viewport width.
fn fit_scale(area: Rect) -> f64 {
    let earth_diameter = 2.0 * earth_r();
    earth_diameter / (EARTH_SCREEN_FRACTION * f64::from(area.width).max(1.0))
}

/// Draw the scene into `buf`. The camera sits in (and the objects live on) the root frame,
/// so for this overview the camera-frame position is just the object's position; a body's
/// on-screen radius is its `r_eq / scale`. Bodies whose centre is off-screen still draw the
/// part of their disc that lands inside `area` (the renderer culls per-dot).
fn render_scene(store: &SceneStore, area: Rect, buf: &mut Buffer) {
    if area.width == 0 || area.height == 0 {
        return;
    }
    let snap = store.snapshot();
    let scale = fit_scale(area);
    let cx = f64::from(area.width) / 2.0;
    let cy = f64::from(area.height) / 2.0;

    let mut pts: Vec<(f64, f64)> = Vec::new();
    for obj in snap.objects() {
        let col = cx + obj.state.position.x / scale;
        let row = cy - obj.state.position.y / scale; // +y up → rows grow down
        let radius = obj.shape.map_or(0.0, |s| s.ellipsoid.r_eq() / scale);
        if radius >= 0.75 {
            fill_disc(&mut pts, col, row, radius);
        } else {
            pts.push((col, row));
        }
    }
    BrailleRenderer::new().draw_points(&pts, area, buf);
}

/// Tessellate a filled disc of `radius` cells centred at `(cx, cy)` into braille-resolution
/// points, aspect-corrected so it renders round. Samples every sub-cell dot (½ cell wide,
/// ¼ cell tall).
fn fill_disc(pts: &mut Vec<(f64, f64)>, cx: f64, cy: f64, radius: f64) {
    let r_row = radius / CELL_ASPECT;
    let mut col = cx - radius;
    while col <= cx + radius {
        let mut row = cy - r_row;
        while row <= cy + r_row {
            let dx = (col - cx) / radius;
            let dy = (row - cy) / r_row;
            if dx * dx + dy * dy <= 1.0 {
                pts.push((col, row));
            }
            row += 0.25;
        }
        col += 0.5;
    }
}

fn main() -> io::Result<()> {
    let store = build_scene();
    let mut terminal = ratatui::init();
    let result = run(&mut terminal, &store);
    ratatui::restore();
    result
}

fn run(terminal: &mut ratatui::DefaultTerminal, store: &SceneStore) -> io::Result<()> {
    loop {
        terminal.draw(|frame| {
            let area = frame.area();
            render_scene(store, area, frame.buffer_mut());
        })?;
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press
                    && matches!(key.code, KeyCode::Char('q') | KeyCode::Esc)
                {
                    return Ok(());
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Lit columns on `area`'s LOCAL row `row`, returned as columns local to the area
    // (`0..width`). Buffer indexing is offset by `area.x`/`area.y`, so it's correct for any
    // area origin and the callers' comparisons against `area.width` stay in one space.
    fn lit_columns(buf: &Buffer, area: Rect, row: u16) -> Vec<u16> {
        (0..area.width)
            .filter(|&c| buf[(area.x + c, area.y + row)].symbol() != " ")
            .collect()
    }

    #[test]
    fn scene_has_the_three_bodies() {
        let snap = build_scene().snapshot();
        let mut labels: Vec<&str> = snap.objects().iter().map(|o| o.label.as_ref()).collect();
        labels.sort_unstable();
        assert_eq!(labels, ["Earth", "Moon", "Sun"]);
        assert!(snap.objects().iter().all(|o| o.kind == ObjectKind::Body));
    }

    #[test]
    fn earth_disc_spans_more_than_a_fifth_of_the_width() {
        let store = build_scene();
        let area = Rect::new(0, 0, 120, 40);
        let mut buf = Buffer::empty(area);
        render_scene(&store, area, &mut buf);

        // On the centre row, Earth's filled disc straddles the middle; measure its width.
        let mid = area.height / 2;
        let lit = lit_columns(&buf, area, mid);
        assert!(!lit.is_empty(), "nothing drawn on the centre row");
        // Earth is the cluster around the centre column.
        let centre = area.width / 2;
        let near: Vec<u16> = lit
            .iter()
            .copied()
            .filter(|&x| x.abs_diff(centre) < area.width / 4)
            .collect();
        assert!(
            !near.is_empty(),
            "Earth disc not found near the centre column"
        );
        let span = near.last().unwrap() - near.first().unwrap() + 1;
        assert!(
            f64::from(span) >= 0.2 * f64::from(area.width),
            "Earth disc spans {span} cells of {}, want ≥ 20%",
            area.width
        );
    }

    #[test]
    fn sun_and_moon_are_both_visible() {
        let store = build_scene();
        let area = Rect::new(0, 0, 120, 40);
        let mut buf = Buffer::empty(area);
        render_scene(&store, area, &mut buf);
        let mid = area.height / 2;
        let lit = lit_columns(&buf, area, mid);
        // Sun limb sits left of Earth, Moon sits right of it — so the lit span on the centre
        // row reaches both the left and right thirds of the viewport.
        assert!(
            lit.iter().any(|&x| x < area.width / 3),
            "Sun not visible on the left"
        );
        assert!(
            lit.iter().any(|&x| x > 2 * area.width / 3),
            "Moon not visible on the right"
        );
    }
}
