//! The committed sample replay (`scenes/orbit.json`, produced by `examples/record_demo`) must
//! stay a valid, playable recording — so `cargo run -p orchestrator -- --replay …` keeps working.

use astrotui_core::scene::SceneStore;
use astrotui_wire::Replay;

const ORBIT_JSON: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/scenes/orbit.json");

#[test]
fn sample_orbit_replay_loads_and_plays() {
    let replay = Replay::from_json_path(ORBIT_JSON).expect("orbit.json loads + validates");
    assert_eq!(replay.len(), 72, "one full 72-epoch orbit");
    let (start, end) = replay.time_bounds().expect("non-empty timeline");
    assert!(end > start);

    // Apply a mid-orbit epoch: Earth (fixed) and the probe (moving) are both placed.
    let store = SceneStore::new();
    replay
        .apply_at((start + end) / 2.0, &mut store.writer("replay"))
        .expect("mid-orbit epoch applies");
    let snap = store.snapshot();
    let mut ids: Vec<&str> = snap.objects().iter().map(|o| o.id.as_str()).collect();
    ids.sort_unstable();
    assert_eq!(ids, ["earth", "probe"]);
    // Both carry a shape (so the orchestrator renders them as discs).
    assert!(snap.objects().iter().all(|o| o.shape.is_some()));

    // The probe actually moves: its position differs between the first and a later epoch.
    let pos = |t: f64| {
        let s = SceneStore::new();
        replay.apply_at(t, &mut s.writer("replay")).unwrap();
        s.snapshot()
            .objects()
            .iter()
            .find(|o| o.id.as_str() == "probe")
            .unwrap()
            .state
            .position
    };
    assert_ne!(pos(start), pos((start + end) / 2.0), "the probe orbits");
}
