//! orchestrator — the reference astrotui host.
//!
//! P0 demo (#16): a trivial in-process producer places the Sun, Earth, and Moon as points
//! on a `RootInertial` frame, and the app renders them as a braille overview through the
//! `camera = frame → project → draw` pipeline. Positions are representative samples, not
//! real ephemeris (the ephemeris body-filler is a P3 producer). Press `q` or `Esc` to quit.

use std::io;
use std::time::Duration;

use astrotui_core::render::{Camera, SpaceView};
use astrotui_core::scene::{BodyState, Epoch, ObjectKind, ObjectMeta, SceneStore, Snapshot};
use astrotui_render_braille::BrailleRenderer;
use glam::DVec3;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::Rect;

/// Sun–Earth distance (1 AU) and Earth–Moon distance, in metres — enough to place three
/// recognisable points; not an ephemeris.
const AU: f64 = 1.495_978_707e11;
const EARTH_MOON: f64 = 3.844e8;

/// Build the demo scene: Sun at the origin, Earth at +1 AU, the Moon just beyond it, all as
/// `Body` points on the root inertial frame.
fn build_scene() -> SceneStore {
    let store = SceneStore::new();
    let mut tx = store.writer("demo").begin(Epoch::from_seconds(0.0));
    tx.frame("root", None, BodyState::default());
    tx.object("sun", "root", point(0.0, 0.0), body("Sun"))
        .object("earth", "root", point(AU, 0.0), body("Earth"))
        .object("moon", "root", point(AU + EARTH_MOON, 0.0), body("Moon"));
    tx.commit();
    store
}

fn point(x: f64, y: f64) -> BodyState {
    BodyState {
        position: DVec3::new(x, y, 0.0),
        ..BodyState::default()
    }
}

fn body(label: &'static str) -> ObjectMeta {
    ObjectMeta {
        label: label.into(),
        kind: ObjectKind::Body,
        ..ObjectMeta::default()
    }
}

/// A `RootInertial` overview camera scaled to fit the snapshot's objects into `area`.
fn fit_camera(snap: &Snapshot, area: Rect) -> Camera {
    let max_r = snap
        .objects()
        .iter()
        .map(|o| o.state.position.x.abs().max(o.state.position.y.abs()))
        .fold(0.0_f64, f64::max);
    // Terminal cells are ~2× taller than wide; approximate a square fit.
    let half = (f64::from(area.width).min(f64::from(area.height) * 2.0)) * 0.45;
    let scale = if max_r > 0.0 && half > 0.0 {
        max_r / half
    } else {
        1.0
    };
    Camera::overview("root", scale)
}

fn main() -> io::Result<()> {
    let mut store = build_scene();
    let mut terminal = ratatui::init();
    let result = run(&mut terminal, &mut store);
    ratatui::restore();
    result
}

fn run(terminal: &mut ratatui::DefaultTerminal, store: &mut SceneStore) -> io::Result<()> {
    loop {
        terminal.draw(|frame| {
            let area = frame.area();
            let camera = fit_camera(&store.snapshot(), area);
            let renderer = BrailleRenderer::new();
            frame.render_stateful_widget(SpaceView::new(&camera, &renderer), area, store);
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
    use ratatui::buffer::Buffer;
    use ratatui::widgets::StatefulWidget;

    #[test]
    fn scene_has_the_three_bodies() {
        let snap = build_scene().snapshot();
        let mut labels: Vec<&str> = snap.objects().iter().map(|o| o.label.as_ref()).collect();
        labels.sort_unstable();
        assert_eq!(labels, ["Earth", "Moon", "Sun"]);
        assert!(snap.objects().iter().all(|o| o.kind == ObjectKind::Body));
    }

    #[test]
    fn overview_draws_braille_dots() {
        let mut store = build_scene();
        let area = Rect::new(0, 0, 80, 24);
        let camera = fit_camera(&store.snapshot(), area);
        let mut buf = Buffer::empty(area);
        SpaceView::new(&camera, &BrailleRenderer::new()).render(area, &mut buf, &mut store);

        let drawn = buf.content().iter().filter(|c| c.symbol() != " ").count();
        assert!(
            drawn >= 1,
            "expected braille dots in the overview, found none"
        );
    }
}
