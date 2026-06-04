//! Camera + the per-object render projection pipeline (DESIGN.md §3, §4.4).
//!
//! The load-bearing idea (§3): a camera *is* a reference frame, so projecting an object
//! reduces to asking astrodyn for its state relative to the camera's frame. This skeleton
//! (issue #14):
//!
//! 1. builds an astrodyn [`FrameTree`] from a [`Snapshot`]'s frame records,
//! 2. places each object as a transient child frame at its in-frame position,
//! 3. reads `compute_relative_state(camera, object).trans.position` — the resolve-once
//!    primitive of §4.4, letting astrodyn do all the frame math, and
//! 4. orthographically projects that camera-frame position to a terminal cell.
//!
//! Drawing those cells is the renderer's job (#15). Perspective + seamless log-zoom +
//! angular-size LOD arrive in P1; frame *orientation* (rotating frames, `DQuat`→JEOD quat)
//! wires in with the Moon path in P2. This skeleton handles translational frame placement
//! with an orthographic camera — enough to validate camera=frame + projection on the
//! `RootInertial` overview.

use std::collections::HashMap;

use astrodyn_frames::{FrameTree, RefFrameKind, RefFrameState, RefFrameTrans};
use glam::DVec3;
use ratatui::layout::Rect;

use crate::scene::{FrameId, ObjectId, Snapshot};

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

/// An object projected into the viewport.
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectedPoint {
    /// Which object.
    pub id: ObjectId,
    /// Cell column/row in the viewport.
    pub cell: (u16, u16),
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
    let Some((mut tree, ids)) = build_tree(snap) else {
        return Vec::new();
    };
    let Some(&cam_id) = ids.get(&camera.frame) else {
        return Vec::new();
    };

    // Add every object as a transient child frame at its in-frame position, so astrodyn
    // composes the full chain to the camera frame for us (no hand-rolled frame math).
    let mut object_frames = Vec::with_capacity(snap.objects().len());
    for obj in snap.objects() {
        let Some(&parent) = ids.get(&obj.frame) else {
            continue;
        };
        let aid = tree.add_child(
            parent,
            format!("__obj:{}", obj.id),
            RefFrameKind::Body,
            trans_state(obj.state.position, obj.state.velocity),
        );
        object_frames.push((obj.id.clone(), aid));
    }

    let mut out = Vec::with_capacity(object_frames.len());
    for (id, aid) in object_frames {
        let pos_cam = tree.compute_relative_state(cam_id, aid).trans.position;
        if let Some(cell) = project_orthographic(pos_cam, camera.scale, area) {
            out.push(ProjectedPoint { id, cell, pos_cam });
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
/// down −Z, screen +x is camera +x (right), screen +y is camera +y (up). `None` if the
/// point falls outside `area`. Cell aspect (terminal cells ≈ 2:1) is refined with the
/// backends in #15; here one cell spans `metres_per_cell` on both axes.
fn project_orthographic(pos_cam: DVec3, metres_per_cell: f64, area: Rect) -> Option<(u16, u16)> {
    let cx = f64::from(area.x) + f64::from(area.width) / 2.0;
    let cy = f64::from(area.y) + f64::from(area.height) / 2.0;
    let col = cx + pos_cam.x / metres_per_cell;
    let row = cy - pos_cam.y / metres_per_cell; // camera +y is up; rows grow downward

    let left = f64::from(area.x);
    let top = f64::from(area.y);
    let right = f64::from(area.x) + f64::from(area.width);
    let bottom = f64::from(area.y) + f64::from(area.height);
    if col < left || col >= right || row < top || row >= bottom {
        return None;
    }
    // In-bounds, non-negative, < u16::MAX viewport extents → the truncation is exact.
    Some((col as u16, row as u16))
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
        assert_eq!(origin.cell, (10, 5)); // centre of the 20x10 area
        let right = pts.iter().find(|p| p.id.as_str() == "right").unwrap();
        assert_eq!(right.cell, (12, 5)); // +4 m / 2 m-per-cell = +2 cells in +x
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
        assert_eq!(probe.cell, (200 + 105, 5));

        // From a camera riding "ship", the same probe is only 5 m in x.
        let pts = project_points(
            &snap,
            &Camera::overview("ship", 1.0),
            Rect::new(0, 0, 40, 10),
        );
        let probe = pts.iter().find(|p| p.id.as_str() == "probe").unwrap();
        assert_eq!(probe.pos_cam.x, 5.0);
        assert_eq!(probe.cell, (25, 5));
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
}
