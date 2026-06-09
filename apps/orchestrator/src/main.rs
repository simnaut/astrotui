//! orchestrator — the reference astrotui host.
//!
//! Demo (#16, extended): a trivial in-process producer places the Sun, Earth, and Moon on
//! a `RootInertial` frame at their **real radii and real mutual distances** (Earth–Moon
//! 384 400 km, Earth–Sun 1 AU), and the app views them through a **perspective camera**
//! 350 000 km from Earth. Nothing here is size-stylized: the camera distance plus its field
//! of view is what makes Earth span ~40% of the width — the apparent size falls out of where
//! the camera sits, not a hand-picked fraction.
//!
//! The only thing arranged is where the camera points: Earth is the nearest body, with the
//! Sun (1 AU beyond it) and the Moon (384 400 km beyond it) both on the far side, nudged a
//! couple of degrees off the view axis so all three read as distinct discs. The honest,
//! counter-intuitive payoff of true scale: the Sun, 109× Earth's radius, renders *far*
//! smaller than the nearby Earth (~10% vs 40% of the width) because it is ~430× farther
//! away; the Moon, beyond Earth and tiny, is smaller still (~5%).
//!
//! Core's projection is orthographic (DESIGN.md §4.4 skeleton); the small perspective
//! projector here previews P1's perspective + seamless log-zoom (#18) and the angular-size
//! point→disc LOD (#19), which will subsume it into the core `Renderer`. Press `q`/`Esc` to
//! quit.
//!
//! **Replay mode** (#22, DESIGN §8(b)): `orchestrator --replay <file.json>` plays a recorded
//! scene series ([`Replay`]) instead of the live demo, driving the `SceneWriter` at
//! wall-clock-controlled sim time. The same widget/render path renders it; object positions are
//! shown in their native frame (full frame-aware projection lands with #18).

use std::io;
use std::time::{Duration, Instant};

use astrodyn_planet::{PlanetShape, EARTH, MOON, SUN};
use astrodyn_quantities::{FrameUid, RootInertial};
use astrotui_core::producer::Producer;
use astrotui_core::render::Renderer;
use astrotui_core::scene::{
    BodyShape, BodyState, Epoch, ObjectKind, ObjectMeta, SceneStore, SceneWriter,
};
use astrotui_render_braille::BrailleRenderer;
use astrotui_wire::{Replay, ReplayError};
use glam::DVec3;
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::Rect;

/// Terminal cells are roughly twice as tall as wide; correct for it so discs look round.
const CELL_ASPECT: f64 = 2.0;
/// How far the camera sits from Earth (m): 350 000 km — a close vantage, just inside the
/// Moon's orbital radius, so Earth (the nearest body) looms large and the far-off Sun is
/// small.
const CAM_DISTANCE_M: f64 = 3.5e8;
/// Earth's apparent diameter as a fraction of the viewport width (> 0.2, per the brief). The
/// camera's field of view is derived from this and [`CAM_DISTANCE_M`] — Earth is *framed*,
/// not resized.
const EARTH_SCREEN_FRACTION: f64 = 0.40;

/// Mean Earth–Sun distance (m): 1 astronomical unit.
const EARTH_SUN_M: f64 = 1.495_978_707e11;
/// Mean Earth–Moon distance (m).
const EARTH_MOON_M: f64 = 3.844e8;

/// Framing tilt of the Sun off the camera→Earth axis (rad, ≈ −1.83°), toward screen-left.
/// A couple of degrees — the Sun stays a true 1 AU away, just not dead behind Earth.
const SUN_FRAMING_RAD: f64 = -0.031_97;
/// Framing tilt of the Moon off the axis (rad, ≈ +3.49°), toward screen-right. Larger than
/// the Sun's because the Moon is so much closer, it needs more angle to clear Earth's disc.
const MOON_FRAMING_RAD: f64 = 0.060_97;

/// `tan(fov_x / 2)`: the half-width field of view that frames Earth at
/// [`EARTH_SCREEN_FRACTION`] from [`CAM_DISTANCE_M`]. Earth's on-screen diameter fraction is
/// `r_eq / (distance · tan(fov/2))`, so solving for the tangent fixes the framing.
fn fov_half_tan() -> f64 {
    EARTH.r_eq() / (CAM_DISTANCE_M * EARTH_SCREEN_FRACTION)
}

/// The in-process [`Producer`] for the demo scene — the first `Producer` impl (DESIGN §4.1).
///
/// Places the Sun–Earth–Moon on the root inertial frame at **real radii and real distances**.
/// Earth sits at the origin (nearest the camera); the Sun is a true 1 AU beyond it and the
/// Moon a true 384 400 km beyond it, both on the far side and tilted a couple of degrees off
/// the view axis so the camera frames three separate discs with Earth dominant. The bodies are
/// scene *objects* (the demo exercises the producer seam, not the frame-document consumer).
struct DemoProducer;

impl Producer for DemoProducer {
    fn populate(&self, w: &mut SceneWriter) {
        let root = FrameUid::of::<RootInertial>();
        let mut tx = w.begin(Epoch::from_seconds(0.0));
        tx.frame(root.clone(), None, BodyState::default());
        tx.object(
            "earth",
            root.clone(),
            point(DVec3::ZERO),
            body("Earth", EARTH),
        )
        .object(
            "sun",
            root.clone(),
            // 1 AU beyond Earth (+z, away from the camera), tilted toward screen-left.
            point(axis_offset(EARTH_SUN_M, SUN_FRAMING_RAD, false)),
            body("Sun", SUN),
        )
        .object(
            "moon",
            root,
            // 384 400 km beyond Earth (+z, far side), tilted toward screen-right.
            point(axis_offset(EARTH_MOON_M, MOON_FRAMING_RAD, false)),
            body("Moon", MOON),
        );
        tx.commit();
    }
}

/// Build the demo scene by running [`DemoProducer`] into a fresh store.
fn build_scene() -> SceneStore {
    let store = SceneStore::new();
    DemoProducer.populate(&mut store.writer("demo"));
    store
}

/// A world position `distance` m from the origin, tilted `az` rad in the x–z plane off the
/// view axis (the root z-axis through the camera and Earth). `toward_camera` places it on the
/// −z (near) side of Earth; otherwise the +z (far) side.
fn axis_offset(distance: f64, az: f64, toward_camera: bool) -> DVec3 {
    let z = distance * az.cos();
    DVec3::new(distance * az.sin(), 0.0, if toward_camera { -z } else { z })
}

fn point(position: DVec3) -> BodyState {
    BodyState {
        position,
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

/// Draw the scene into `buf` through the perspective camera. The camera sits at
/// `−CAM_DISTANCE_M` on the root z-axis looking toward +z, so a world point's camera-frame
/// position is just `position + (0, 0, CAM_DISTANCE_M)`. A body's on-screen radius is its true
/// `r_eq` divided by the world width spanned by half the screen at that depth — so nearer
/// bodies loom larger and the distant Sun shrinks. Off-screen dots are culled per-dot by the
/// renderer.
fn render_scene(store: &SceneStore, area: Rect, buf: &mut Buffer) {
    if area.width == 0 || area.height == 0 {
        return;
    }
    let snap = store.snapshot();
    let half_tan = fov_half_tan();
    let (w, h) = (f64::from(area.width), f64::from(area.height));

    let mut pts: Vec<(f64, f64)> = Vec::new();
    for obj in snap.objects() {
        let cam = obj.state.position + DVec3::new(0.0, 0.0, CAM_DISTANCE_M);
        if cam.z <= 1.0 {
            continue; // at or behind the camera
        }
        // World metres spanning half the viewport width at this depth.
        let half_width_m = cam.z * half_tan;
        let col = w / 2.0 + (cam.x / half_width_m) * (w / 2.0);
        let row = h / 2.0 - (cam.y / half_width_m) * (w / 2.0) / CELL_ASPECT; // +y up
        let radius = obj
            .shape
            .map_or(0.0, |s| s.ellipsoid.r_eq() / half_width_m * (w / 2.0));
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

/// Sim seconds advanced per real second of playback (1× — wall-clock time).
const PLAYBACK_SPEED: f64 = 1.0;

/// Map wall-clock `elapsed_secs` since playback began to a replay sim time, offset to the
/// recording's `start_simtime` and scaled by [`PLAYBACK_SPEED`]. Pure — the loop's clock.
fn replay_sim_time(start_simtime: f64, elapsed_secs: f64) -> f64 {
    start_simtime + elapsed_secs * PLAYBACK_SPEED
}

/// The replay file path from `--replay <path>` in `args` (skipping argv[0]), if present.
fn replay_arg(mut args: impl Iterator<Item = String>) -> Option<String> {
    while let Some(a) = args.next() {
        if a == "--replay" {
            return args.next();
        }
    }
    None
}

fn main() -> io::Result<()> {
    // `--replay <file>` plays a recorded scene series (DESIGN §8(b)); otherwise the live demo.
    if let Some(path) = replay_arg(std::env::args().skip(1)) {
        run_replay(&path)
    } else {
        let store = build_scene();
        let mut terminal = ratatui::init();
        let result = run(&mut terminal, &store);
        ratatui::restore();
        result
    }
}

/// Load a recorded replay and play it in the TUI at wall-clock-controlled sim time. The viz
/// outlives the recording: past the last cue the final snapshot simply stays on screen (§4).
fn run_replay(path: &str) -> io::Result<()> {
    // Preserve the original error: a file `Io` error keeps its kind (e.g. `NotFound`); decode/
    // validation errors become an `io::Error` carrying the `ReplayError` as their source.
    let replay = Replay::from_json_path(path).map_err(|e| match e {
        ReplayError::Io(io) => io,
        other => io::Error::other(other),
    })?;
    let store = SceneStore::new();
    let mut terminal = ratatui::init();
    let result = run_replay_loop(&mut terminal, &replay, &store);
    ratatui::restore();
    result
}

fn run_replay_loop(
    terminal: &mut ratatui::DefaultTerminal,
    replay: &Replay,
    store: &SceneStore,
) -> io::Result<()> {
    let start = replay.time_bounds().map_or(0.0, |(s, _)| s);
    let begun = Instant::now();
    loop {
        let t = replay_sim_time(start, begun.elapsed().as_secs_f64());
        // Per-epoch apply re-validates the targeted row, incl. the dangling-parent guard that
        // whole-series `validate()` does not run — so it *can* still fail on a bad row. Surface
        // it loudly (the error becomes the `io::Error` source) rather than rendering a half scene.
        replay
            .apply_at(t, &mut store.writer("replay"))
            .map_err(io::Error::other)?;
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
    fn demo_producer_populates_a_writer() {
        // The demo's body placement is a Producer; running it into a writer fills the store.
        let store = SceneStore::new();
        DemoProducer.populate(&mut store.writer("demo"));
        assert_eq!(store.snapshot().objects().len(), 3);
    }

    #[test]
    fn sun_and_moon_keep_their_real_distances() {
        // The framing tilt must not perturb the true Earth–Sun / Earth–Moon distances.
        let snap = build_scene().snapshot();
        let dist = |id: &str| {
            snap.objects()
                .iter()
                .find(|o| o.id.as_str() == id)
                .unwrap()
                .state
                .position
                .length()
        };
        assert!((dist("sun") - EARTH_SUN_M).abs() < 1.0);
        assert!((dist("moon") - EARTH_MOON_M).abs() < 1.0);
    }

    #[test]
    fn earth_disc_dominates_the_view() {
        let store = build_scene();
        let area = Rect::new(0, 0, 120, 40);
        let mut buf = Buffer::empty(area);
        render_scene(&store, area, &mut buf);

        // On the centre row, Earth's filled disc straddles the middle; measure its width.
        let mid = area.height / 2;
        let lit = lit_columns(&buf, area, mid);
        assert!(!lit.is_empty(), "nothing drawn on the centre row");
        // Earth is the cluster in the central half — the Sun (far left) and Moon (far right)
        // sit outside this window, so we measure Earth alone.
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
        // Earth is framed at 40% of the width; require well over the 20% brief to lock that in.
        assert!(
            f64::from(span) >= 0.3 * f64::from(area.width),
            "Earth disc spans {span} cells of {}, want ≥ 30%",
            area.width
        );
    }

    #[test]
    fn distant_sun_renders_smaller_than_the_nearer_earth() {
        // The whole point of true scale: the Sun (109× Earth's radius) is so far that it
        // subtends a smaller disc than the nearby Earth.
        let store = build_scene();
        let area = Rect::new(0, 0, 120, 40);
        let mut buf = Buffer::empty(area);
        render_scene(&store, area, &mut buf);
        let mid = area.height / 2;
        let lit = lit_columns(&buf, area, mid);
        let centre = area.width / 2;

        let cluster_span = |keep: &dyn Fn(u16) -> bool| -> u16 {
            let cols: Vec<u16> = lit.iter().copied().filter(|&x| keep(x)).collect();
            assert!(!cols.is_empty(), "expected a body in this region");
            cols.last().unwrap() - cols.first().unwrap() + 1
        };
        let sun = cluster_span(&|x| x < area.width / 4);
        let earth = cluster_span(&|x| x.abs_diff(centre) < area.width / 4);
        assert!(
            sun < earth,
            "Sun spans {sun} cells but Earth only {earth}; the distant Sun should look smaller"
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
        // Sun sits left of Earth, Moon right of it — and Earth (40% wide) stays inside the
        // central half, so the outer quarters can only be lit by the Sun and the Moon.
        assert!(
            lit.iter().any(|&x| x < area.width / 4),
            "Sun not visible on the left"
        );
        assert!(
            lit.iter().any(|&x| x > 3 * area.width / 4),
            "Moon not visible on the right"
        );
    }

    #[test]
    fn replay_sim_time_offsets_and_scales() {
        // 1× playback: sim time = recording start + elapsed wall seconds (concrete literals, so
        // a change to PLAYBACK_SPEED or the formula actually fails this).
        assert_eq!(PLAYBACK_SPEED, 1.0);
        assert_eq!(replay_sim_time(100.0, 0.0), 100.0);
        assert_eq!(replay_sim_time(100.0, 2.5), 102.5);
        assert_eq!(replay_sim_time(0.0, 10.0), 10.0);
    }

    #[test]
    fn replay_arg_extracts_the_path() {
        let got = replay_arg(["--replay", "descent.json"].iter().map(|s| s.to_string()));
        assert_eq!(got.as_deref(), Some("descent.json"));
        assert_eq!(replay_arg(["--replay"].iter().map(|s| s.to_string())), None); // missing value
        assert_eq!(replay_arg(std::iter::empty()), None); // no flag → live demo
    }
}
