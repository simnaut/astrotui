//! Camera + the per-object render projection pipeline (DESIGN.md §3, §4.4).
//!
//! The load-bearing idea (§3): a camera *is* a reference frame, so projecting an object
//! reduces to asking astrodyn for its state relative to the camera's frame. This skeleton
//! (issue #14):
//!
//! 1. builds an astrodyn [`FrameTree`] from a [`Snapshot`]'s frame records,
//! 2. resolves ONE transform per occupied frame via `compute_relative_state(camera, F)`
//!    (§4.4 step 2), letting astrodyn do the frame math,
//! 3. applies that transform to every object in the frame
//!    (`pos_cam = origin + R_{F→cam} · p`), and
//! 4. orthographically projects each camera-frame position to a terminal cell.
//!
//! Drawing those cells is the renderer's job (#15). Perspective + seamless log-zoom +
//! angular-size LOD arrive in P1; frame *orientation* (rotating frames, `DQuat`→JEOD quat)
//! wires in with the Moon path in P2. This skeleton handles translational frame placement
//! with an orthographic camera — enough to validate camera=frame + projection on the
//! `RootInertial` overview.

use std::collections::HashMap;

use astrodyn_frames::{FrameTree, RefFrameKind, RefFrameState, RefFrameTrans};
use glam::{DMat3, DVec3};
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::widgets::StatefulWidget;

use crate::scene::{FrameId, ObjectId, SceneStore, Snapshot};

/// The eye. In this skeleton it is a scene frame to sit in plus an orthographic scale; the
/// full `Camera` (target, log-zoom, up, fov — DESIGN.md §4.4) lands with the camera presets
/// and seamless zoom in P1.
#[derive(Clone, Debug)]
pub struct Camera {
    /// The scene frame the eye sits in / is oriented by.
    pub frame: FrameId,
    /// Orthographic scale: metres per terminal cell. Replaced by log-zoom in P1.
    pub scale: f64,
}

impl Camera {
    /// A scene overview anchored in `frame` (the root inertial frame), at `metres_per_cell`
    /// orthographic scale.
    #[must_use]
    pub fn overview(frame: impl Into<FrameId>, metres_per_cell: f64) -> Self {
        Self {
            frame: frame.into(),
            scale: metres_per_cell,
        }
    }
}

/// An object projected into the viewport, in **fractional** cell coordinates so a backend
/// can rasterize at its own resolution (e.g. braille's 2×4 sub-cell dot grid). `(col, row)`
/// are measured from the viewport's top-left in cells; `col` grows right, `row` grows down.
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectedPoint {
    /// Which object.
    pub id: ObjectId,
    /// Fractional cell column (grows right).
    pub col: f64,
    /// Fractional cell row (grows down).
    pub row: f64,
    /// Position in the camera frame (metres) — retained for depth ordering / LOD later.
    pub pos_cam: DVec3,
}

/// Project a snapshot's objects into `area` as seen from `camera` — DESIGN.md §4.4 steps
/// 1–4, point-only. Objects whose frame is unknown, or that fall outside `area`, are
/// omitted. Returns one [`ProjectedPoint`] per visible object, in `snap.objects()` order.
#[must_use]
pub fn project_points(snap: &Snapshot, camera: &Camera, area: Rect) -> Vec<ProjectedPoint> {
    if camera.scale <= 0.0 || area.width == 0 || area.height == 0 {
        return Vec::new();
    }
    let Some((tree, ids)) = build_tree(snap) else {
        return Vec::new();
    };
    let Some(&cam_id) = ids.get(&camera.frame) else {
        return Vec::new();
    };

    // Resolve ONE transform per occupied frame and apply it to every object in that frame
    // (DESIGN.md §4.4 step 2). `compute_relative_state(cam, F)` gives F's origin in camera
    // coordinates and the camera→F rotation; transposing that rotation maps an object's
    // in-frame position `p` into camera coordinates: pos_cam = origin + R_{F→cam} · p.
    let mut by_frame: HashMap<usize, (DVec3, DMat3)> = HashMap::new();
    let mut out = Vec::with_capacity(snap.objects().len());
    for obj in snap.objects() {
        let Some(&frame_id) = ids.get(&obj.frame) else {
            continue;
        };
        let (origin, r_frame_to_cam) = *by_frame.entry(frame_id).or_insert_with(|| {
            let s = tree.compute_relative_state(cam_id, frame_id);
            (s.trans.position, s.rot.t_parent_this.transpose())
        });
        let pos_cam = origin + r_frame_to_cam * obj.state.position;
        if let Some((col, row)) = project_orthographic(pos_cam, camera.scale, area) {
            out.push(ProjectedPoint {
                id: obj.id.clone(),
                col,
                row,
                pos_cam,
            });
        }
    }
    out
}

/// Build an astrodyn `FrameTree` from the snapshot's frame records, returning it alongside
/// a map from scene [`FrameId`] to the arena's `usize` frame id. Frames are added
/// parent-before-child; frames whose parent is missing are dropped. `None` if there are no
/// frames. Frame *kind* and *orientation* are simplified here (all `Inertial`, identity
/// rotation) — sufficient for the translational overview; rotating frames land in P2.
fn build_tree(snap: &Snapshot) -> Option<(FrameTree, HashMap<FrameId, usize>)> {
    let mut tree = FrameTree::new();
    let mut ids: HashMap<FrameId, usize> = HashMap::new();
    let mut remaining: Vec<_> = snap.frames().iter().collect();

    let mut progress = true;
    while progress && !remaining.is_empty() {
        progress = false;
        remaining.retain(|fr| match &fr.parent {
            None => {
                // Root state is the inertial origin (identity); any supplied state is ignored.
                ids.insert(
                    fr.id.clone(),
                    tree.add_root(fr.id.to_string(), RefFrameKind::Inertial),
                );
                progress = true;
                false
            }
            Some(parent) => match ids.get(parent) {
                Some(&parent_id) => {
                    let aid = tree.add_child(
                        parent_id,
                        fr.id.to_string(),
                        RefFrameKind::Inertial,
                        trans_state(fr.state.position, fr.state.velocity),
                    );
                    ids.insert(fr.id.clone(), aid);
                    progress = true;
                    false
                }
                None => true, // parent not added yet (or missing) — retry / drop
            },
        });
    }

    if ids.is_empty() {
        None
    } else {
        Some((tree, ids))
    }
}

/// A translational-only frame state (identity rotation) at the given position/velocity.
fn trans_state(position: DVec3, velocity: DVec3) -> RefFrameState {
    RefFrameState {
        trans: RefFrameTrans { position, velocity },
        ..RefFrameState::default()
    }
}

/// Orthographic projection of a camera-frame position onto the viewport: the eye looks
/// down −Z, screen +x is camera +x (right), screen +y is camera +y (up). Returns
/// **fractional** cell coordinates (so a backend can rasterize at sub-cell resolution), or
/// `None` if the point falls outside `area`. One cell spans `metres_per_cell` on both axes;
/// cell aspect (terminal cells ≈ 2:1) is a backend concern.
fn project_orthographic(pos_cam: DVec3, metres_per_cell: f64, area: Rect) -> Option<(f64, f64)> {
    let cx = f64::from(area.x) + f64::from(area.width) / 2.0;
    let cy = f64::from(area.y) + f64::from(area.height) / 2.0;
    let col = cx + pos_cam.x / metres_per_cell;
    let row = cy - pos_cam.y / metres_per_cell; // camera +y is up; rows grow downward

    // Cull non-finite coordinates first: a NaN/∞ in `pos_cam` would slip through the
    // bounds check below (every comparison with NaN is false) and cast to a bogus cell.
    if !col.is_finite() || !row.is_finite() {
        return None;
    }
    let left = f64::from(area.x);
    let top = f64::from(area.y);
    let right = f64::from(area.x) + f64::from(area.width);
    let bottom = f64::from(area.y) + f64::from(area.height);
    if col < left || col >= right || row < top || row >= bottom {
        return None;
    }
    Some((col, row))
}

/// A rendering backend: rasterizes projected points into a ratatui [`Buffer`]. Backends
/// (braille / color-cell / graphics — DESIGN.md §5.1) live in their own crates and
/// implement this trait, keeping `astrotui-core` backend-agnostic.
pub trait Renderer {
    /// Draw `points` (fractional cell coordinates within `area`, as produced by
    /// [`project_points`]) into `buf`, rasterizing at the backend's own resolution.
    fn draw_points(&self, points: &[(f64, f64)], area: Rect, buf: &mut Buffer);
}

/// The astrotui widget: projects a [`SceneStore`]'s latest snapshot through `camera` and
/// rasterizes it with `renderer`. The renderer is injected so the host picks the backend
/// (capability-based auto-detect arrives in P3); camera presets + log-zoom arrive in P1.
pub struct SpaceView<'a> {
    camera: &'a Camera,
    renderer: &'a dyn Renderer,
}

impl<'a> SpaceView<'a> {
    /// Build a view that renders `camera`'s perspective with `renderer`.
    #[must_use]
    pub fn new(camera: &'a Camera, renderer: &'a dyn Renderer) -> Self {
        Self { camera, renderer }
    }
}

impl StatefulWidget for SpaceView<'_> {
    type State = SceneStore;

    fn render(self, area: Rect, buf: &mut Buffer, state: &mut SceneStore) {
        let snapshot = state.snapshot();
        let points: Vec<(f64, f64)> = project_points(&snapshot, self.camera, area)
            .into_iter()
            .map(|p| (p.col, p.row))
            .collect();
        self.renderer.draw_points(&points, area, buf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{BodyState, Epoch, ObjectMeta, SceneStore};

    fn area() -> Rect {
        Rect::new(0, 0, 20, 10) // centre at (10, 5)
    }
    fn at(x: f64, y: f64) -> BodyState {
        BodyState {
            position: DVec3::new(x, y, 0.0),
            ..BodyState::default()
        }
    }

    // Build a one-frame ("root") scene with the given objects, then snapshot it.
    fn scene(objects: &[(&str, BodyState)]) -> std::sync::Arc<crate::scene::Snapshot> {
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame("root", None, BodyState::default());
        for (id, st) in objects {
            tx.object(*id, "root", *st, ObjectMeta::default());
        }
        tx.commit();
        store.snapshot()
    }

    #[test]
    fn projects_to_viewport_centre_and_offsets() {
        let snap = scene(&[("origin", at(0.0, 0.0)), ("right", at(4.0, 0.0))]);
        let cam = Camera::overview("root", 2.0); // 2 m per cell
        let pts = project_points(&snap, &cam, area());

        let origin = pts.iter().find(|p| p.id.as_str() == "origin").unwrap();
        assert_eq!((origin.col, origin.row), (10.0, 5.0)); // centre of the 20x10 area
        let right = pts.iter().find(|p| p.id.as_str() == "right").unwrap();
        assert_eq!((right.col, right.row), (12.0, 5.0)); // +4 m / 2 m-per-cell = +2 cells in +x
    }

    #[test]
    fn camera_relative_state_composes_through_child_frame() {
        // A child frame offset +100 m in x under root; an object +5 m in x within it.
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame("root", None, BodyState::default())
            .frame("ship", Some("root".into()), at(100.0, 0.0))
            .object("probe", "ship", at(5.0, 0.0), ObjectMeta::default());
        tx.commit();
        let snap = store.snapshot();

        // From the root camera, the probe sits at 105 m in x.
        let pts = project_points(
            &snap,
            &Camera::overview("root", 1.0),
            Rect::new(0, 0, 400, 10),
        );
        let probe = pts.iter().find(|p| p.id.as_str() == "probe").unwrap();
        assert_eq!(probe.pos_cam.x, 105.0);
        assert_eq!((probe.col, probe.row), (305.0, 5.0)); // centre 200 + 105 m

        // From a camera riding "ship", the same probe is only 5 m in x.
        let pts = project_points(
            &snap,
            &Camera::overview("ship", 1.0),
            Rect::new(0, 0, 40, 10),
        );
        let probe = pts.iter().find(|p| p.id.as_str() == "probe").unwrap();
        assert_eq!(probe.pos_cam.x, 5.0);
        assert_eq!((probe.col, probe.row), (25.0, 5.0)); // centre 20 + 5 m
    }

    #[test]
    fn one_transform_per_frame_applies_to_all_its_objects() {
        // Two objects share an offset child frame; each picks up that frame's transform.
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame("root", None, BodyState::default())
            .frame("ship", Some("root".into()), at(100.0, 0.0))
            .object("nose", "ship", at(1.0, 0.0), ObjectMeta::default())
            .object("tail", "ship", at(-1.0, 0.0), ObjectMeta::default());
        tx.commit();
        let snap = store.snapshot();

        let pts = project_points(
            &snap,
            &Camera::overview("root", 1.0),
            Rect::new(0, 0, 400, 10),
        );
        let nose = pts.iter().find(|p| p.id.as_str() == "nose").unwrap();
        let tail = pts.iter().find(|p| p.id.as_str() == "tail").unwrap();
        assert_eq!(nose.pos_cam.x, 101.0); // 100 (frame) + 1 (in-frame)
        assert_eq!(tail.pos_cam.x, 99.0); // 100 (frame) - 1 (in-frame)
    }

    #[test]
    fn non_finite_positions_are_culled() {
        let snap = scene(&[
            ("ok", at(0.0, 0.0)),
            (
                "nan",
                BodyState {
                    position: DVec3::new(f64::NAN, 0.0, 0.0),
                    ..BodyState::default()
                },
            ),
            (
                "inf",
                BodyState {
                    position: DVec3::new(f64::INFINITY, 0.0, 0.0),
                    ..BodyState::default()
                },
            ),
        ]);
        let pts = project_points(&snap, &Camera::overview("root", 1.0), area());
        let ids: Vec<&str> = pts.iter().map(|p| p.id.as_str()).collect();
        assert_eq!(ids, ["ok"]); // NaN/∞ objects are dropped, not mapped to (0,0)
    }

    #[test]
    fn offscreen_objects_are_culled() {
        let snap = scene(&[("near", at(0.0, 0.0)), ("far", at(1_000.0, 0.0))]);
        let pts = project_points(&snap, &Camera::overview("root", 1.0), area());
        let ids: Vec<&str> = pts.iter().map(|p| p.id.as_str()).collect();
        assert_eq!(ids, ["near"]); // "far" is off the 20-wide viewport
    }

    #[test]
    fn empty_or_unknown_camera_frame_yields_nothing() {
        let snap = scene(&[("a", at(0.0, 0.0))]);
        assert!(project_points(&snap, &Camera::overview("nope", 1.0), area()).is_empty());
        assert!(project_points(&snap, &Camera::overview("root", 0.0), area()).is_empty());
        let empty = SceneStore::new().snapshot();
        assert!(project_points(&empty, &Camera::overview("root", 1.0), area()).is_empty());
    }

    /// Records the points handed to a renderer, so we can check what `SpaceView` projected.
    #[derive(Default)]
    struct Recorder(std::cell::RefCell<Vec<(f64, f64)>>);
    impl Renderer for Recorder {
        fn draw_points(&self, points: &[(f64, f64)], _area: Rect, _buf: &mut Buffer) {
            self.0.borrow_mut().extend_from_slice(points);
        }
    }

    #[test]
    fn space_view_projects_snapshot_to_the_renderer() {
        let mut store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame("root", None, BodyState::default())
            .object("origin", "root", at(0.0, 0.0), ObjectMeta::default())
            .object("right", "root", at(4.0, 0.0), ObjectMeta::default());
        tx.commit();

        let cam = Camera::overview("root", 2.0);
        let recorder = Recorder::default();
        let a = area();
        let mut buf = Buffer::empty(a);
        SpaceView::new(&cam, &recorder).render(a, &mut buf, &mut store);

        let mut got = recorder.0.into_inner();
        got.sort_by(|x, y| x.0.total_cmp(&y.0));
        assert_eq!(got, vec![(10.0, 5.0), (12.0, 5.0)]); // same projection as project_points
    }
}
