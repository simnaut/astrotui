//! Record a small **moving** scene as a replay file (#22).
//!
//! Builds a [`SceneSeries`]: Earth fixed at the origin (a stable reference disc) and a
//! Moon-sized "probe" orbiting it in the x–z plane, then JSON-encodes it. The positions are
//! tuned for the orchestrator's demo camera (eye ~3.5e8 m back looking −Z, framing Earth at
//! ~40% width), so the recording renders right out of the box:
//!
//! ```text
//! cargo run -p orchestrator --example record_demo            # writes scenes/orbit.json
//! cargo run -p orchestrator -- --replay apps/orchestrator/scenes/orbit.json
//! ```
//!
//! Pass an output path to write elsewhere: `--example record_demo -- /tmp/scene.json`.

use std::io;

use astrodyn_frame_doc::{
    CanonicalRotation, Conventions, DocHeader, FrameRecord, Origin, SeriesBuilder, TransRecord,
    SCHEMA_VERSION,
};
use astrodyn_planet::{PlanetShape, EARTH, MOON};
use astrodyn_quantities::{FrameUid, RootInertial};
use astrotui_wire::{
    Json, ObjectEpochRow, ObjectKindWire, ObjectRecord, ObjectSegment, SceneSeries, ShapeRecord,
    WireCodec,
};

/// Number of epochs (one full probe orbit).
const EPOCHS: usize = 72;
/// Seconds of sim time between epochs (0.25 s → a ~18 s orbit at 1× playback).
const CADENCE_S: f64 = 0.25;
/// Probe orbit radius (m) — well inside the camera's framed half-width so it stays on screen.
const ORBIT_R_M: f64 = 1.2e7;

fn identity() -> CanonicalRotation {
    CanonicalRotation::Quat([1.0, 0.0, 0.0, 0.0])
}

/// The static root-frame record stamped at sim time `t`.
fn root_record(t: f64) -> FrameRecord {
    FrameRecord {
        name: "root".into(),
        uid_index: 0,
        parent: None,
        epoch: Some(t),
        trans: TransRecord {
            position: [0.0; 3],
            velocity: [0.0; 3],
        },
        rotation: identity(),
        ang_vel_this: [0.0; 3],
        origin: Origin::Injected,
    }
}

/// A wire shape record from an `astrodyn_planet` ellipsoid.
fn shape_of(s: PlanetShape) -> ShapeRecord {
    ShapeRecord {
        name: s.name.to_string(),
        mu: s.mu,
        r_eq: s.r_eq(),
        r_pol: s.r_pol(),
        flat_coeff: s.flat_coeff,
    }
}

/// A body object on the root frame (uid index 0) at `pos`, with the given ellipsoid shape.
fn body(id: &str, label: &str, pos: [f64; 3], shape: PlanetShape) -> ObjectRecord {
    ObjectRecord {
        id: id.into(),
        label: label.into(),
        frame_index: 0,
        kind: ObjectKindWire::Body,
        trans: TransRecord {
            position: pos,
            velocity: [0.0; 3],
        },
        rotation: identity(),
        shape: Some(shape_of(shape)),
        path: None,
    }
}

/// Earth fixed at the origin + a probe orbiting it in the x–z plane (so it sweeps left↔right and
/// pulses in size as it nears/recedes from the eye), over one full revolution.
pub fn build_orbit_series() -> SceneSeries {
    let root = FrameUid::of::<RootInertial>();
    let header = DocHeader {
        schema_version: SCHEMA_VERSION,
        conventions: Conventions::current(),
        simtime: 0.0,
        tai_tjt_at_epoch: 0.0,
    };
    let mut frames = SeriesBuilder::new(header, vec![root]);
    let mut epochs = Vec::with_capacity(EPOCHS);
    for i in 0..EPOCHS {
        let t = i as f64 * CADENCE_S;
        frames.push_epoch(t, vec![root_record(t)]); // root is static; complete row each epoch
        let theta = std::f64::consts::TAU * (i as f64) / (EPOCHS as f64);
        let probe = [ORBIT_R_M * theta.cos(), 0.0, ORBIT_R_M * theta.sin()];
        epochs.push(ObjectEpochRow {
            simtime: t,
            objects: vec![
                body("earth", "Earth", [0.0, 0.0, 0.0], EARTH),
                body("probe", "Moon", probe, MOON),
            ],
        });
    }
    SceneSeries {
        frames: frames.finish(),
        objects: vec![ObjectSegment {
            start_simtime: 0.0,
            epochs,
        }],
    }
}

fn main() -> io::Result<()> {
    let series = build_orbit_series();
    series
        .validate()
        .expect("generated series must validate (it's the keyframe handshake)");
    let bytes = Json
        .encode(&series)
        .expect("JSON encode of a validated series cannot fail");

    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| format!("{}/scenes/orbit.json", env!("CARGO_MANIFEST_DIR")));
    if let Some(dir) = std::path::Path::new(&path).parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&path, &bytes)?;
    eprintln!("wrote {EPOCHS} epochs ({} bytes) to {path}", bytes.len());
    Ok(())
}
