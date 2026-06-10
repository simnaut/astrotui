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
//! 4. **perspective**-projects each camera-frame position from the dollied eye to a terminal
//!    cell (a [`LogZoom`] log-distance along the view axis, #18).
//!
//! Drawing those cells is the renderer's job (#15). Angular-size **LOD** (point → ellipsoid →
//! DEM mesh) arrives in #19; frame *orientation* (rotating frames, `DQuat`→JEOD quat) wired in
//! with #77. The projection is numerically safe across ~12 orders of magnitude because it works
//! in **camera-relative** coordinates (DESIGN.md §4.4, §5.2).

use std::collections::{HashMap, HashSet};

use astrodyn_frames::{FrameTree, RefFrameRot, RefFrameState, RefFrameTrans};
use astrodyn_quantities::frame_identity::topocentric_site_frame_uid;
use astrodyn_quantities::{
    BodyFrame, FrameUid, JeodQuat, Lvlh, Ned, Planet, PlanetFixed, PlanetInertial, RootInertial,
    Vehicle,
};
use glam::{DMat3, DVec3};
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::widgets::StatefulWidget;

use crate::scene::{BodyShape, BodyState, ObjectId, SceneStore, Snapshot};

/// Where the camera looks — resolved per render into a forward view axis in the camera frame's
/// own coordinates (DESIGN.md §4.4). The frame sets origin + orientation; the *target* sets the
/// view axis the eye dollies along (log-zoom is #18; angular-size LOD is #19).
#[derive(Clone, Debug, PartialEq)]
pub enum CameraTarget {
    /// Look at the origin of a frame, named by [`FrameUid`] (the classic "view that frame").
    FrameOrigin(FrameUid),
    /// Track a scene object by id; the view axis is recomputed from its position each frame.
    Object(ObjectId),
    /// A fixed bearing — a direction in the camera frame's own coordinates (need not be unit;
    /// normalized on use). Independent of scene contents.
    Bearing(DVec3),
}

impl CameraTarget {
    /// The forward view-axis (unit, camera-frame coordinates), or `None` if it can't be formed.
    /// `Bearing` resolves itself; `FrameOrigin`/`Object` resolve from `look_at_cam` — the
    /// target's position in camera coordinates, which the render pass computes from the
    /// `FrameTree` (#18) and supplies here. A `None`/zero point or zero bearing yields `None`
    /// (a degenerate look axis — e.g. the target coincides with the eye), surfaced by the
    /// caller, never silently pointed somewhere arbitrary.
    #[must_use]
    pub fn forward(&self, look_at_cam: Option<DVec3>) -> Option<DVec3> {
        match self {
            CameraTarget::Bearing(dir) => dir.try_normalize(),
            CameraTarget::FrameOrigin(_) | CameraTarget::Object(_) => {
                look_at_cam.and_then(DVec3::try_normalize)
            }
        }
    }
}

/// The camera's "which way is up" reference, in the camera frame's coordinates. Combined with
/// the view axis to build an orthonormal [`ViewBasis`]; the basis is **gimbal-guarded** for the
/// degenerate case where up is parallel to the view axis (see [`Camera::view_basis`]).
#[derive(Clone, Debug, PartialEq)]
pub enum UpHint {
    /// The camera frame's +Z axis — the sensible default for most frames.
    FrameUp,
    /// An explicit up direction in camera-frame coordinates (normalized on use).
    Direction(DVec3),
}

impl UpHint {
    /// The (un-normalized) up direction in camera-frame coordinates.
    fn vector(&self) -> DVec3 {
        match self {
            UpHint::FrameUp => DVec3::Z,
            UpHint::Direction(v) => *v,
        }
    }
}

/// A right-handed orthonormal camera basis, in the camera frame's coordinates: the eye looks
/// along `forward`, `right` is screen-right, `up` is screen-up. `right × up == -forward` — the
/// eye looks down its own −Z, the screen convention [`project_perspective`] uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewBasis {
    /// Screen-right (unit).
    pub right: DVec3,
    /// Screen-up (unit).
    pub up: DVec3,
    /// The view axis the eye looks along (unit).
    pub forward: DVec3,
}

/// Squared length below which the `forward × up` cross product is treated as a gimbal-lock
/// degeneracy. Inputs are unit, so this is `sin²θ` between them — ~0.057° of separation.
const GIMBAL_EPS_SQ: f64 = 1e-6;

/// Build a right-handed orthonormal [`ViewBasis`] from a view axis + up hint, **gimbal-guarded**.
/// `forward` need not be unit; a (near-)zero `forward` falls back to the frame's −Z look axis,
/// and an up hint (near-)parallel to `forward` falls back to an alternate axis — so the result
/// is always orthonormal and finite.
fn orthonormal_view(forward: DVec3, up_hint: DVec3) -> ViewBasis {
    let f = forward.try_normalize().unwrap_or(DVec3::NEG_Z);
    let up0 = up_hint.try_normalize().unwrap_or(DVec3::Z);
    // right = forward × up (with f = −Z, up = +Y this is +X = screen-right).
    let mut right = f.cross(up0);
    if right.length_squared() < GIMBAL_EPS_SQ {
        // up ∥ forward (gimbal lock): pick an alternate up least aligned with the view axis.
        let alt = if f.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
        right = f.cross(alt);
    }
    let right = right.normalize();
    let up = right.cross(f); // re-orthogonalized; unit since right ⟂ f and both are unit.
    ViewBasis {
        right,
        up,
        forward: f,
    }
}

/// A **log-distance dolly**: the eye sits [`distance`](Self::distance) metres back from the view
/// anchor along the view axis (DESIGN.md §3/§4.4/§5.2). Stored as `log10(distance)` so equal
/// [`nudge`](Self::nudge)s are equal *screen* steps across the ~12 orders of magnitude between
/// interplanetary cruise and a lunar touchdown — a linear store would crowd every near scale into
/// the bottom ULPs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LogZoom {
    /// log10 of the eye→anchor distance in metres.
    log10_distance: f64,
}

/// `log10(distance)` is clamped to this band so [`LogZoom::distance`] is always finite and
/// strictly positive: `10^±307` is comfortably inside `f64`'s finite, normal range. The band
/// still spans 600+ orders of magnitude — far more than the ~12 the camera needs.
const LOG_ZOOM_BOUND: f64 = 307.0;

impl LogZoom {
    /// A dolly `metres` from the anchor. A non-finite or non-positive distance clamps to the
    /// smallest in-band dolly so the eye never coincides with the anchor (which would collapse
    /// the view); express a *degenerate* view via the target/forward path, not a zero distance.
    #[must_use]
    pub fn from_distance(metres: f64) -> Self {
        if metres.is_finite() && metres > 0.0 {
            Self::from_log10(metres.log10())
        } else {
            Self {
                log10_distance: -LOG_ZOOM_BOUND,
            }
        }
    }

    /// Construct directly from `log10(distance)`, **clamped** to `±`[`LOG_ZOOM_BOUND`] so
    /// `distance()` can't overflow to `∞` (non-finite → [`Default`]).
    #[must_use]
    pub fn from_log10(x: f64) -> Self {
        if x.is_finite() {
            Self {
                log10_distance: x.clamp(-LOG_ZOOM_BOUND, LOG_ZOOM_BOUND),
            }
        } else {
            Self::default()
        }
    }

    /// Eye→anchor distance in metres (= `10^log10`), always finite and `> 0`.
    #[must_use]
    pub fn distance(&self) -> f64 {
        10f64.powf(self.log10_distance)
    }

    /// `log10` of the distance — the raw dolly parameter (for UI sliders / smooth interpolation).
    #[must_use]
    pub fn log10(&self) -> f64 {
        self.log10_distance
    }

    /// Dolly by `decades` (+ moves the eye away, − moves it closer). Smooth zoom is a constant
    /// rate of change in this parameter — the seamlessness the single-camera model needs.
    #[must_use]
    pub fn nudge(self, decades: f64) -> Self {
        Self::from_log10(self.log10_distance + decades)
    }
}

impl Default for LogZoom {
    /// 1e7 m (10 000 km) — a near-orbit vantage that frames a planet-sized body.
    fn default() -> Self {
        Self {
            log10_distance: 7.0,
        }
    }
}

/// Default full horizontal field of view (radians) — 40°.
const DEFAULT_FOV_RAD: f64 = 0.698_131_7;

/// Near-plane distance (m): objects at or behind it are culled (a non-positive depth would flip
/// the perspective divide and alias to the wrong side).
const NEAR_PLANE_M: f64 = 1.0;

/// The eye. A scene frame to sit in (astrodyn #659 identity), a [`CameraTarget`] view axis, an
/// [`UpHint`], a [`LogZoom`] log-distance dolly, and a field of view. One **seamless log-zoom
/// perspective camera** spanning ~12 orders of magnitude (DESIGN.md §3/§4.4/§5.2); angular-size
/// **LOD** arrives in #19. The frame/target/up model (#17) is the base it builds on.
#[derive(Clone, Debug)]
pub struct Camera {
    /// Identity of the scene frame the eye sits in / is oriented by (astrodyn #659).
    pub frame: FrameUid,
    /// What the eye looks at — the view axis.
    pub target: CameraTarget,
    /// Which way is up when building the view basis.
    pub up: UpHint,
    /// Log-distance dolly of the eye along the view axis.
    pub zoom: LogZoom,
    /// Full **horizontal** field of view, radians — the angle spanned across the viewport
    /// **width**. Cells are square (`project_perspective` scales both axes by half the width), so
    /// the vertical extent follows from the viewport's aspect; terminal cell aspect (≈ 2:1) stays
    /// a backend concern (#19).
    pub fov: f64,
}

impl Camera {
    /// A scene overview anchored in the frame named by `frame` (e.g.
    /// `FrameUid::of::<RootInertial>()`), looking at that frame's origin with frame-up, at the
    /// default dolly + field of view. The general-frame form of [`Camera::solar_overview`]; tune
    /// the view by setting the public `zoom`/`fov` fields.
    #[must_use]
    pub fn overview(frame: FrameUid) -> Self {
        Self::in_frame(frame)
    }

    /// Common preset body: sit in `frame`, look at its origin, frame-up, default dolly + fov.
    fn in_frame(frame: FrameUid) -> Self {
        Self {
            target: CameraTarget::FrameOrigin(frame.clone()),
            frame,
            up: UpHint::FrameUp,
            zoom: LogZoom::default(),
            fov: DEFAULT_FOV_RAD,
        }
    }

    /// **Solar-system overview** — the inertial root (`RootInertial`). Earth→Jupiter cruise.
    #[must_use]
    pub fn solar_overview() -> Self {
        Self::in_frame(FrameUid::of::<RootInertial>())
    }

    /// **Inertial chase** — a planet's non-rotating inertial frame (`PlanetInertial<P>`). Orbits.
    #[must_use]
    pub fn inertial_chase<P: Planet>() -> Self {
        Self::in_frame(FrameUid::of::<PlanetInertial<P>>())
    }

    /// **Body-fixed** — a planet's rotating body-fixed frame (`PlanetFixed<P>`). Ground track,
    /// lunar approach.
    #[must_use]
    pub fn body_fixed<P: Planet>() -> Self {
        Self::in_frame(FrameUid::of::<PlanetFixed<P>>())
    }

    /// **Orbit-relative** — a chief vehicle's LVLH frame (`Lvlh<V>`). Nadir / ram-pointed.
    #[must_use]
    pub fn orbit_relative<V: Vehicle>() -> Self {
        Self::in_frame(FrameUid::of::<Lvlh<V>>())
    }

    /// **Vehicle local NED** — a moving vehicle's north-east-down frame (`Ned<V>`).
    #[must_use]
    pub fn vehicle_ned<V: Vehicle>() -> Self {
        Self::in_frame(FrameUid::of::<Ned<V>>())
    }

    /// **Onboard** — a vehicle's body frame (`BodyFrame<V>`). Cockpit / sensor boresight.
    #[must_use]
    pub fn onboard<V: Vehicle>() -> Self {
        Self::in_frame(FrameUid::of::<BodyFrame<V>>())
    }

    /// **Local horizon** — a site-anchored topocentric (ENU) frame on planet `P` (landing site,
    /// ground station). Unlike the other presets, a topocentric frame's identity is **value-keyed**
    /// by `(planet, site)`: `FrameUid::of::<Topocentric<P>>()` is keyed by planet alone, so every
    /// site on `P` would collide — astrodyn mints site-distinguished uids through the one shared
    /// [`topocentric_site_frame_uid`] (astrodyn #688/#696), which a producer and the viz both call
    /// so they converge byte-for-byte. `site` is a stable site key (e.g. `"KSC-LC39A"`); the
    /// geodetic anchor itself rides the frame's transform in the scene, not the identity.
    #[must_use]
    pub fn local_horizon<P: Planet>(site: &str) -> Self {
        Self::in_frame(topocentric_site_frame_uid(P::NAME, site))
    }

    /// Build the orthonormal [`ViewBasis`] for a resolved `forward` view axis (in camera-frame
    /// coordinates), applying this camera's [`UpHint`]. Gimbal-guarded — see [`orthonormal_view`].
    #[must_use]
    pub fn view_basis(&self, forward: DVec3) -> ViewBasis {
        orthonormal_view(forward, self.up.vector())
    }
}

/// An object projected into the viewport, in **fractional** cell coordinates **local to the
/// render area** so a backend can rasterize at its own resolution (e.g. braille's 2×4
/// sub-cell dot grid). `(col, row)` are measured from the area's top-left: `(0, 0)` is the
/// top-left cell, `col` grows right, `row` grows down — independent of where the area sits
/// in the buffer (the backend adds the `area.x`/`area.y` offset when it writes cells).
///
/// `#[non_exhaustive]`: this is an *output* of [`project_points`] — consumers read its
/// fields, never construct it — so it is marked non-exhaustive to keep adding projection
/// outputs (depth, angular size, …) from being a breaking change for downstream crates.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct ProjectedPoint {
    /// Which object.
    pub id: ObjectId,
    /// Fractional cell column (grows right).
    pub col: f64,
    /// Fractional cell row (grows down).
    pub row: f64,
    /// Position in the camera frame (metres) — retained for depth ordering / LOD later.
    pub pos_cam: DVec3,
    /// The object's body axes expressed in **camera-frame** coordinates (body → camera frame),
    /// composed from the frame→camera rotation and the object's attitude. NOT screen/view space —
    /// the view rotation (the [`ViewBasis`]) is applied to positions only; a consumer wanting
    /// object→view composes it itself. Consumed by oriented-ellipsoid LOD in P1/P2.
    pub att_cam: DMat3,
}

/// Diagnostics from a projection pass: what could **not** be rendered, so the host can
/// surface it instead of presenting a silently blank screen (DESIGN.md §4.4 — loud, never
/// silent). Off-screen objects are normal culling and are *not* reported here.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RenderReport {
    /// Objects whose `frame` uid is absent from the resolved `FrameTree`.
    pub orphan_objects: Vec<ObjectId>,
    /// Frames dropped while building the tree: a root-ineligible identity, or a child whose
    /// parent never resolved (a dangling or cyclic parent).
    pub dropped_frames: Vec<FrameUid>,
    /// Set when the camera's own frame is absent from the tree — nothing can be drawn.
    pub unresolved_camera_frame: Option<FrameUid>,
}

impl RenderReport {
    /// `true` when every object and frame resolved and the camera frame was found.
    #[must_use]
    pub fn is_clean(&self) -> bool {
        self.orphan_objects.is_empty()
            && self.dropped_frames.is_empty()
            && self.unresolved_camera_frame.is_none()
    }
}

/// Project a snapshot's objects into `area` as seen from `camera` — DESIGN.md §4.4 steps
/// 1–4, point-only. Returns one [`ProjectedPoint`] per visible object (in `snap.objects()`
/// order) **and** a [`RenderReport`] of what could not be rendered — orphan objects, dropped
/// frames, and an absent camera frame are *surfaced*, never silently culled (§4.4). Objects
/// that simply fall outside `area` are omitted as normal culling (not reported).
#[must_use]
pub fn project_points(
    snap: &Snapshot,
    camera: &Camera,
    area: Rect,
) -> (Vec<ProjectedPoint>, RenderReport) {
    let (points, report, _ctx) = project_core(snap, camera, area);
    (points, report)
}

/// The view context the perspective pass resolved: the dollied `eye`, the screen `view` basis,
/// `tan(fov/2)`, and the viewport half-width (cells). Shared between [`project_points`] and the
/// LOD pass ([`project_bodies`]) so the angular-size + oblate-silhouette math reuses exactly the
/// projector's geometry.
struct ViewCtx {
    eye: DVec3,
    view: ViewBasis,
    tan_half: f64,
    half: f64,
}

/// The shared frame-composition + perspective projection (DESIGN.md §4.4). Returns the projected
/// points, the [`RenderReport`], and the [`ViewCtx`] (`None` on a degenerate lens / empty canvas /
/// unresolvable camera frame, where nothing projects).
fn project_core(
    snap: &Snapshot,
    camera: &Camera,
    area: Rect,
) -> (Vec<ProjectedPoint>, RenderReport, Option<ViewCtx>) {
    // The lens must be a sane angle and there must be a canvas. (`tan(fov/2)` finite & > 0 rules
    // out a degenerate or ≥180° fov.)
    let tan_half = (camera.fov * 0.5).tan();
    if !(camera.fov > 0.0 && camera.fov < std::f64::consts::PI && tan_half.is_finite())
        || area.width == 0
        || area.height == 0
    {
        return (Vec::new(), RenderReport::default(), None);
    }
    let (built, dropped_frames) = build_tree(snap);
    let mut report = RenderReport {
        dropped_frames,
        ..RenderReport::default()
    };
    let Some((tree, ids)) = built else {
        return (Vec::new(), report, None);
    };
    let Some(&cam_id) = ids.get(&camera.frame) else {
        // The eye's own frame isn't in the tree: nothing can be drawn — surface it loudly.
        report.unresolved_camera_frame = Some(camera.frame.clone());
        return (Vec::new(), report, None);
    };

    // Pass 1 — frame composition (DESIGN.md §4.4 step 2), unchanged: resolve ONE transform per
    // occupied frame and every object's `pos_cam`/`att_cam` (camera-FRAME coordinates).
    // `compute_relative_state(cam, F)` gives F's origin in camera coords + the camera→F rotation;
    // transposing maps an in-frame position `p` to camera coords: pos_cam = origin + R_{F→cam}·p.
    let mut by_frame: HashMap<usize, (DVec3, DMat3)> = HashMap::new();
    let mut pending: Vec<(ObjectId, DVec3, DMat3)> = Vec::with_capacity(snap.objects().len());
    for obj in snap.objects() {
        let Some(&frame_id) = ids.get(&obj.frame) else {
            report.orphan_objects.push(obj.id.clone());
            continue;
        };
        let (origin, r_frame_to_cam) = *by_frame.entry(frame_id).or_insert_with(|| {
            let s = tree.compute_relative_state(cam_id, frame_id);
            (s.trans.position, s.rot.t_parent_this.transpose())
        });
        let pos_cam = origin + r_frame_to_cam * obj.state.position;
        // Object body axes in camera coords: (frame→cam) ∘ (body→frame). The attitude is the
        // body orientation in its native frame (parent→this), so the transpose of its
        // parent→this matrix is body→frame.
        let body_to_frame = JeodQuat::from_glam(obj.state.attitude)
            .left_quat_to_transformation()
            .transpose();
        let att_cam = r_frame_to_cam * body_to_frame;
        pending.push((obj.id.clone(), pos_cam, att_cam));
    }

    // Resolve the view anchor + axis in camera coords (DESIGN.md §4.4 step 4). The target sets
    // what the eye looks at; the eye then dollies back along the view axis by `zoom.distance()`.
    let look_at_cam: Option<DVec3> = match &camera.target {
        CameraTarget::Bearing(_) => None,
        // The target frame's origin in camera coords (ZERO when it's the camera's own frame).
        CameraTarget::FrameOrigin(uid) => ids.get(uid).copied().map(|fid| {
            by_frame
                .entry(fid)
                .or_insert_with(|| {
                    let s = tree.compute_relative_state(cam_id, fid);
                    (s.trans.position, s.rot.t_parent_this.transpose())
                })
                .0
        }),
        CameraTarget::Object(id) => pending.iter().find(|(i, ..)| i == id).map(|&(_, p, _)| p),
    };
    // `−Z` fallback (the established orthographic look axis) when the target can't form a
    // direction — e.g. a camera on its own frame origin (`look_at = ZERO`). Only `Bearing(ZERO)`
    // stays genuinely degenerate (and `forward` then yields `None` → `−Z` here too, harmless).
    let forward = camera.target.forward(look_at_cam).unwrap_or(DVec3::NEG_Z);
    let anchor = look_at_cam.unwrap_or(DVec3::ZERO);
    let view = camera.view_basis(forward);
    let eye = anchor - forward * camera.zoom.distance();

    // Pass 2 — perspective-project each object's pos_cam from the dollied eye.
    let (w, h) = (f64::from(area.width), f64::from(area.height));
    let mut out = Vec::with_capacity(pending.len());
    for (id, pos_cam, att_cam) in pending {
        if let Some((col, row)) = project_perspective(pos_cam, eye, &view, tan_half, w, h) {
            out.push(ProjectedPoint {
                id,
                col,
                row,
                pos_cam,
                att_cam,
            });
        }
    }
    let ctx = ViewCtx {
        eye,
        view,
        tan_half,
        half: w / 2.0,
    };
    (out, report, Some(ctx))
}

/// Which on-screen representation a body draws at, chosen by angular size (DESIGN.md §5.2). The
/// DEM-mesh level is P2 (#26+) and out of scope here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lod {
    /// Sub-resolution: a single sub-cell point.
    Point,
    /// Resolvable: a filled ellipse silhouette (true intensity shading is the color/graphics
    /// backends, #20+).
    Ellipsoid,
}

/// `Point → Ellipsoid` once the on-screen equatorial radius reaches this (cells, col metric).
const LOD_GROW_CELLS: f64 = 1.5;
/// `Ellipsoid → Point` once it shrinks below this (cells). Lower than [`LOD_GROW_CELLS`]: the gap
/// is the **hysteresis** band, so a body whose size jitters across the boundary keeps its prior
/// representation instead of flipping every frame (DESIGN §5.2). 0.75 ≈ the smallest disc that
/// reads as more than a dot; the 2:1 band rides out a streaming producer's depth jitter.
const LOD_SHRINK_CELLS: f64 = 0.75;

/// Per-object LOD memory across frames — the hysteresis state. Deliberately **not** in the
/// (immutable, lock-free) [`Snapshot`]/[`SceneStore`]: it is render-pass-local mutable state the
/// host owns, one per live view, threaded into [`project_bodies`] each frame.
#[derive(Clone, Debug, Default)]
pub struct LodMemory {
    prev: HashMap<ObjectId, Lod>,
}

impl LodMemory {
    /// An empty memory (every body classifies fresh on the first frame).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Choose this frame's [`Lod`] for `id` from its on-screen radius and the previous frame's
    /// choice, applying the grow/shrink hysteresis; records the result. With no prior, classify
    /// against the band midpoint so the first frame is deterministic and centered.
    fn select(&mut self, id: &ObjectId, screen_radius: f64) -> Lod {
        let next = match self.prev.get(id) {
            Some(Lod::Ellipsoid) => {
                if screen_radius < LOD_SHRINK_CELLS {
                    Lod::Point
                } else {
                    Lod::Ellipsoid
                }
            }
            Some(Lod::Point) => {
                if screen_radius >= LOD_GROW_CELLS {
                    Lod::Ellipsoid
                } else {
                    Lod::Point
                }
            }
            None => {
                if screen_radius >= (LOD_GROW_CELLS + LOD_SHRINK_CELLS) * 0.5 {
                    Lod::Ellipsoid
                } else {
                    Lod::Point
                }
            }
        };
        self.prev.insert(id.clone(), next);
        next
    }

    /// Drop ids not seen this frame, so a removed/culled body neither leaks memory nor resurrects
    /// a stale [`Lod`]. A body that briefly leaves the view loses its hysteresis state and
    /// re-seeds from the band midpoint on return — acceptable, as an off-screen body has no
    /// current size to hysterese against.
    fn retain_seen(&mut self, seen: &HashSet<ObjectId>) {
        self.prev.retain(|id, _| seen.contains(id));
    }
}

/// An object projected into the viewport **with** its angular-size LOD resolved (DESIGN §5.2).
/// Carries everything [`ProjectedPoint`] does plus the chosen [`Lod`] and the on-screen ellipse
/// geometry (cells, col metric) for the [`Renderer`].
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct ProjectedBody {
    /// Which object.
    pub id: ObjectId,
    /// Fractional cell column of the body centre (grows right).
    pub col: f64,
    /// Fractional cell row of the body centre (grows down).
    pub row: f64,
    /// Position in the camera frame (metres).
    pub pos_cam: DVec3,
    /// Body axes in camera-frame coordinates (see [`ProjectedPoint::att_cam`]).
    pub att_cam: DMat3,
    /// The representation chosen for this frame.
    pub lod: Lod,
    /// On-screen **equatorial** radius (cells) — the ellipse's semi-major axis (⟂ the projected
    /// polar axis). Also the LOD selector's input.
    pub screen_radius: f64,
    /// On-screen semi-**minor** axis (cells), along the projected polar axis: `screen_radius` for
    /// a sphere/pole-on view, shrinking to `screen_radius·r_pol/r_eq` edge-on.
    pub semi_minor: f64,
    /// Angle of the major axis from screen-right (radians, down-positive cell frame).
    pub tilt: f64,
}

impl ProjectedBody {
    /// The backend draw primitive for this body (DESIGN §5.1). `Point` LOD → a dot; `Ellipsoid`
    /// LOD → the oriented filled ellipse.
    #[must_use]
    pub fn to_render_body(&self) -> RenderBody {
        let kind = match self.lod {
            Lod::Point => RenderKind::Point,
            Lod::Ellipsoid => RenderKind::Ellipsoid {
                semi_major: self.screen_radius,
                semi_minor: self.semi_minor,
                tilt: self.tilt,
            },
        };
        RenderBody {
            col: self.col,
            row: self.row,
            kind,
        }
    }
}

/// Project a snapshot's objects with **angular-size LOD** selection + hysteresis (DESIGN §5.2).
/// Like [`project_points`] but classifies each *shaped* object [`Point`](Lod::Point) /
/// [`Ellipsoid`](Lod::Ellipsoid) using `mem` (mutated in place — the cross-frame hysteresis
/// state) and emits a [`ProjectedBody`] carrying the LOD + on-screen ellipse geometry. Shapeless
/// objects are always points. Returns the same [`RenderReport`] as [`project_points`].
#[must_use]
pub fn project_bodies(
    snap: &Snapshot,
    camera: &Camera,
    area: Rect,
    mem: &mut LodMemory,
) -> (Vec<ProjectedBody>, RenderReport) {
    let (points, report, ctx) = project_core(snap, camera, area);
    let Some(ctx) = ctx else {
        return (Vec::new(), report);
    };
    // One pass to index shapes by id, so the per-body lookup below is O(1) (BodyShape is Copy).
    let shapes: HashMap<ObjectId, BodyShape> = snap
        .objects()
        .iter()
        .filter_map(|o| o.shape.map(|s| (o.id.clone(), s)))
        .collect();

    let mut seen = HashSet::with_capacity(points.len());
    let mut out = Vec::with_capacity(points.len());
    for p in points {
        seen.insert(p.id.clone());
        let shape = shapes.get(&p.id);
        // Angular size: equatorial radius over the world width spanned by half the screen at this
        // depth (reuses the projector's depth/tan_half/half — same metric as `project_perspective`).
        let screen_radius = shape.map_or(0.0, |s| {
            let depth = (p.pos_cam - ctx.eye).dot(ctx.view.forward);
            if depth > 0.0 {
                s.ellipsoid.r_eq() / (depth * ctx.tan_half) * ctx.half
            } else {
                0.0
            }
        });
        let lod = mem.select(&p.id, screen_radius);
        let (semi_minor, tilt) = match (lod, shape) {
            (Lod::Ellipsoid, Some(s)) => {
                oblate_silhouette(s, &p.att_cam, &ctx.view, p.pos_cam - ctx.eye, screen_radius)
            }
            // Point LOD (or shapeless): the ellipse fields are unused.
            _ => (screen_radius, 0.0),
        };
        out.push(ProjectedBody {
            id: p.id,
            col: p.col,
            row: p.row,
            pos_cam: p.pos_cam,
            att_cam: p.att_cam,
            lod,
            screen_radius,
            semi_minor,
            tilt,
        });
    }
    mem.retain_seen(&seen);
    (out, report)
}

/// On-screen silhouette of an **oriented oblate spheroid**: an ellipse. Returns its semi-**minor**
/// axis (cells, along the projected polar axis) and the major-axis `tilt` (radians, from
/// screen-right in the down-positive cell frame). The semi-major axis is `screen_radius` (the
/// equatorial radius, ⟂ the projected polar).
///
/// The outline of a spheroid `(r_eq, r_eq, r_pol)` viewed at angle `φ` between its polar axis and
/// the line of sight has minor axis `r_eq·sqrt(1 − e²·sin²φ)` (e² = ellipsoid eccentricity²):
/// pole-on (φ→0) → a circle; edge-on (φ→90°) → squashed to `r_eq·r_pol/r_eq = r_pol`.
fn oblate_silhouette(
    shape: &BodyShape,
    att_cam: &DMat3,
    view: &ViewBasis,
    to_body: DVec3,
    screen_radius: f64,
) -> (f64, f64) {
    // Body polar axis (body +Z) in camera coords, and the line of sight to the body.
    let Some(polar) = (*att_cam * DVec3::Z).try_normalize() else {
        return (screen_radius, 0.0);
    };
    let Some(ray) = to_body.try_normalize() else {
        return (screen_radius, 0.0);
    };
    let cos_phi = polar.dot(ray);
    let sin2_phi = (1.0 - cos_phi * cos_phi).max(0.0);
    let semi_minor = screen_radius
        * (1.0 - shape.ellipsoid.e_ellip_sq() * sin2_phi)
            .max(0.0)
            .sqrt();

    // Project the polar axis onto the screen (down-positive cell frame: +up → −row). The major
    // axis is perpendicular to it; if the polar axis is ~along the view (pole-on), the silhouette
    // is ~circular and the tilt is irrelevant.
    let px = polar.dot(view.right);
    let py = -polar.dot(view.up);
    let tilt = if px.hypot(py) < 1e-6 {
        0.0
    } else {
        py.atan2(px) + std::f64::consts::FRAC_PI_2
    };
    (semi_minor, tilt)
}

/// A built frame arena: the astrodyn `FrameTree` plus the map from scene [`FrameUid`] to the
/// arena's `usize` frame id.
type FrameArena = (FrameTree, HashMap<FrameUid, usize>);

/// Build an astrodyn `FrameTree` from the snapshot's frame records. Returns the arena
/// (`None` if no frame resolved) **and** the uids of frames that were dropped — a
/// root-ineligible identity, or a child whose parent never resolved — so the caller can
/// surface them (#76). Frames are stamped by their uid (astrodyn #659) and added
/// parent-before-child. Frame *orientation* is simplified here (translational placement,
/// identity rotation) — sufficient for the overview; rotating frames land in P2.
fn build_tree(snap: &Snapshot) -> (Option<FrameArena>, Vec<FrameUid>) {
    let mut tree = FrameTree::new();
    let mut ids: HashMap<FrameUid, usize> = HashMap::new();
    let mut dropped: Vec<FrameUid> = Vec::new();
    let mut remaining: Vec<_> = snap.frames().iter().collect();

    let mut progress = true;
    while progress && !remaining.is_empty() {
        progress = false;
        remaining.retain(|fr| match &fr.parent {
            None => {
                // A root frame's identity must be root-eligible; drop a malformed root
                // rather than panicking in the render path. Root state is the inertial
                // origin (identity); any supplied state is ignored.
                if !fr.uid.class.may_be_root_or_integ() {
                    dropped.push(fr.uid.clone());
                    return false;
                }
                ids.insert(
                    fr.uid.clone(),
                    tree.add_root_uid(fr.uid.clone(), fr.uid.to_string()),
                );
                progress = true;
                false
            }
            Some(parent) => match ids.get(parent) {
                Some(&parent_id) => {
                    let aid = tree.add_child_uid(
                        parent_id,
                        fr.uid.clone(),
                        fr.uid.to_string(),
                        frame_state(&fr.state),
                        fr.epoch,
                    );
                    ids.insert(fr.uid.clone(), aid);
                    progress = true;
                    false
                }
                None => true, // parent not added yet (or missing) — retry, else dropped below
            },
        });
    }
    // Anything still pending after no further progress had a parent that never resolved
    // (dangling or cyclic) — record it as dropped so the caller can surface it.
    dropped.extend(remaining.iter().map(|fr| fr.uid.clone()));

    if ids.is_empty() {
        (None, dropped)
    } else {
        (Some((tree, ids)), dropped)
    }
}

/// Build an astrodyn `RefFrameState` from a scene `BodyState`, carrying translation **and**
/// rotation. The attitude (a glam `DQuat`, parent→this) converts losslessly to a `JeodQuat`
/// (`from_glam` is the inverse of the wire's `rotation_to_dquat`); the matrix is re-derived
/// from the quaternion (astrodyn RF.04). Angular velocity is zero — `BodyState` carries none.
fn frame_state(s: &BodyState) -> RefFrameState {
    let mut q = JeodQuat::from_glam(s.attitude);
    q.normalize(); // a Matrix→DQuat conversion upstream may drift off unit norm
    RefFrameState {
        trans: RefFrameTrans {
            position: s.position,
            velocity: s.velocity,
        },
        rot: RefFrameRot {
            q_parent_this: q,
            t_parent_this: q.left_quat_to_transformation(),
            ang_vel_this: DVec3::ZERO,
        },
    }
}

/// Perspective-project a camera-frame position from the dollied `eye` looking along
/// `view.forward`, with `tan_half = tan(fov/2)`. Returns **fractional** cell coordinates local to
/// the area (so a backend can rasterize at sub-cell resolution), or `None` if the point is at/
/// behind the near plane, non-finite, or off-area.
///
/// **Numerically safe across ~12 orders of magnitude** (DESIGN.md §4.4/§5.2): `rel = pos_cam −
/// eye` is the difference of two *camera-relative* vectors (both from `compute_relative_state`
/// against the camera frame), so no absolute heliocentric magnitudes are ever differenced —
/// `rel` stays small near the eye even when the dolly distance is ~1e11 m. **Square-cell /
/// aspect-agnostic**: one screen half-*width* of world spans the projection plane at unit depth
/// on both axes, so terminal cell aspect (≈ 2:1) stays a backend concern.
fn project_perspective(
    pos_cam: DVec3,
    eye: DVec3,
    view: &ViewBasis,
    tan_half: f64,
    w: f64,
    h: f64,
) -> Option<(f64, f64)> {
    let rel = pos_cam - eye; // camera-relative — small by construction
    let depth = rel.dot(view.forward);
    if depth <= NEAR_PLANE_M {
        return None; // at/behind the eye → cull (a non-positive divide would flip sides)
    }
    let x = rel.dot(view.right);
    let y = rel.dot(view.up);
    let half = w / 2.0;
    // Coordinates are LOCAL to the area: (0, 0) is the area's top-left.
    let col = half + (x / (depth * tan_half)) * half;
    let row = h / 2.0 - (y / (depth * tan_half)) * half; // +y up → rows down

    // Cull non-finite first: a NaN/∞ would slip through the bounds check below.
    if !col.is_finite() || !row.is_finite() {
        return None;
    }
    if col < 0.0 || col >= w || row < 0.0 || row >= h {
        return None;
    }
    Some((col, row))
}

/// One body to rasterize, its centre in fractional cell coordinates **local to the area**
/// (`(0, 0)` = the area's top-left), as produced by [`project_bodies`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderBody {
    /// Fractional cell column of the centre (grows right).
    pub col: f64,
    /// Fractional cell row of the centre (grows down).
    pub row: f64,
    /// What to draw at `(col, row)`.
    pub kind: RenderKind,
}

/// The drawable form of a body at its chosen [`Lod`] (DESIGN §5.1). Semi-axes are in cells
/// (col metric); the backend aspect-corrects rows for its own cell shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RenderKind {
    /// Sub-resolution: a single sub-cell dot.
    Point,
    /// A filled ellipse silhouette. `semi_major` is the equatorial radius (⟂ the projected polar
    /// axis); `semi_minor` is along the projected polar axis; `tilt` is the major-axis angle
    /// (radians, from screen-right). A circle is `semi_major == semi_minor`. (#20 will shade this
    /// same primitive per-cell.)
    Ellipsoid {
        /// Equatorial on-screen radius (cells).
        semi_major: f64,
        /// Polar-direction on-screen radius (cells).
        semi_minor: f64,
        /// Major-axis angle from screen-right (radians).
        tilt: f64,
    },
}

/// A rendering backend: rasterizes projected bodies into a ratatui [`Buffer`]. Backends
/// (braille / color-cell / graphics — DESIGN.md §5.1) live in their own crates and implement this
/// trait, keeping `astrotui-core` backend-agnostic.
pub trait Renderer {
    /// Draw `bodies` into `buf`, rasterizing at the backend's own resolution. Each body's centre
    /// is in fractional cell coordinates **local to `area`**; the backend offsets by
    /// `area.x`/`area.y` when it writes cells, and renders each [`RenderKind`] at its own fidelity
    /// (braille → silhouette; color/graphics → shaded, #20+).
    fn draw_bodies(&self, bodies: &[RenderBody], area: Rect, buf: &mut Buffer);

    /// Draw bare points — a convenience for point-only callers. Default-implemented as degenerate
    /// [`RenderKind::Point`] bodies fed to [`draw_bodies`](Renderer::draw_bodies), so a backend
    /// only implements `draw_bodies`.
    fn draw_points(&self, points: &[(f64, f64)], area: Rect, buf: &mut Buffer) {
        let bodies: Vec<RenderBody> = points
            .iter()
            .map(|&(col, row)| RenderBody {
                col,
                row,
                kind: RenderKind::Point,
            })
            .collect();
        self.draw_bodies(&bodies, area, buf);
    }
}

/// The stateful render state a [`SpaceView`] mutates each frame: the scene store plus the
/// cross-frame LOD hysteresis [`LodMemory`]. The memory lives here (not in the lock-free
/// `SceneStore`) because it is per-view render state — two cameras of one scene hysterese
/// independently — and `SpaceView` is rebuilt per frame so it cannot hold it itself.
#[derive(Default)]
pub struct ViewState {
    /// The scene to render.
    pub scene: SceneStore,
    /// Per-object LOD memory threaded across frames.
    pub lod: LodMemory,
}

impl ViewState {
    /// Wrap a [`SceneStore`] with fresh (empty) LOD memory.
    #[must_use]
    pub fn new(scene: SceneStore) -> Self {
        Self {
            scene,
            lod: LodMemory::new(),
        }
    }
}

/// The astrotui widget: projects a [`ViewState`]'s latest snapshot through `camera` (a log-zoom
/// perspective camera) with angular-size LOD and rasterizes it with `renderer`. The renderer is
/// injected so the host picks the backend (capability-based auto-detect arrives in P3).
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
    type State = ViewState;

    fn render(self, area: Rect, buf: &mut Buffer, state: &mut ViewState) {
        let snapshot = state.scene.snapshot();
        // The widget draws the LOD'd bodies; a host that wants the unresolved-frame diagnostics
        // (DESIGN §4.4) calls `project_bodies` directly and inspects the [`RenderReport`] (e.g.
        // for a status line). The widget itself has no surface to show them on.
        let (bodies, _report) = project_bodies(&snapshot, self.camera, area, &mut state.lod);
        let render_bodies: Vec<RenderBody> =
            bodies.iter().map(ProjectedBody::to_render_body).collect();
        self.renderer.draw_bodies(&render_bodies, area, buf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{BodyState, Epoch, ObjectMeta, SceneStore};
    use astrodyn_quantities::{Mars, Moon, PlanetFixed, RootInertial};

    fn area() -> Rect {
        Rect::new(0, 0, 20, 10) // centre at (10, 5)
    }
    fn at(x: f64, y: f64) -> BodyState {
        BodyState {
            position: DVec3::new(x, y, 0.0),
            ..BodyState::default()
        }
    }
    // Distinct, real frame identities for the projection tests. `root` is the inertial
    // root; `child` is any distinct child-frame node; `absent` is never placed in a scene.
    fn root() -> FrameUid {
        FrameUid::of::<RootInertial>()
    }
    fn child() -> FrameUid {
        FrameUid::of::<PlanetFixed<Moon>>()
    }
    fn absent() -> FrameUid {
        FrameUid::of::<PlanetFixed<Mars>>()
    }

    fn at3(x: f64, y: f64, z: f64) -> BodyState {
        BodyState {
            position: DVec3::new(x, y, z),
            ..BodyState::default()
        }
    }
    // Build a one-frame (root) scene with the given objects, then snapshot it.
    fn scene(objects: &[(&str, BodyState)]) -> std::sync::Arc<crate::scene::Snapshot> {
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default());
        for (id, st) in objects {
            tx.object(*id, root(), *st, ObjectMeta::default());
        }
        tx.commit();
        store.snapshot()
    }
    // A perspective camera in `frame` with **+Y up** (so screen right=+X, up=+Y — avoids the
    // top-down gimbal of the default frame-up), the eye dollied `dist` m back along −Z, default
    // 40° fov. `tan(fov/2) = tan(20°) ≈ 0.363970`, so an object `x` m off-axis at depth `dist`
    // projects to `col = w/2 + (x/(dist·tan20°))·(w/2)`.
    fn persp(frame: FrameUid, dist: f64) -> Camera {
        Camera {
            up: UpHint::Direction(DVec3::Y),
            zoom: LogZoom::from_distance(dist),
            ..Camera::overview(frame)
        }
    }

    #[test]
    fn log_zoom_round_trips_and_nudges_by_decades() {
        // `powf` rounding varies by libm, so compare with a relative epsilon, never `==`.
        assert!((LogZoom::from_distance(1e7).distance() - 1e7).abs() < 1.0);
        assert!((LogZoom::default().distance() - 1e7).abs() < 1.0);
        // A +1-decade nudge multiplies the distance by 10.
        let z = LogZoom::from_distance(1000.0);
        assert!((z.nudge(1.0).distance() - 10_000.0).abs() < 1e-6);
        assert!((z.nudge(-1.0).distance() - 100.0).abs() < 1e-9);
        assert!((z.log10() - 3.0).abs() < 1e-12);
        // Non-finite / non-positive distance clamps to a finite positive dolly (never zero).
        assert!(LogZoom::from_distance(0.0).distance() > 0.0);
        assert!(LogZoom::from_distance(-5.0).distance() > 0.0);
        assert!(LogZoom::from_distance(f64::NAN).distance().is_finite());
        assert_eq!(LogZoom::from_log10(f64::INFINITY), LogZoom::default());
        // distance() stays finite even for an extreme exponent (the ±bound clamp).
        assert!(LogZoom::from_log10(1e9).distance().is_finite());
        assert!(LogZoom::from_log10(1e9).distance() > 0.0);
    }

    #[test]
    fn projects_to_viewport_centre_and_perspective_offset() {
        let snap = scene(&[("origin", at(0.0, 0.0)), ("right", at(100.0, 0.0))]);
        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), area());

        let origin = pts.iter().find(|p| p.id.as_str() == "origin").unwrap();
        assert_eq!((origin.col, origin.row), (10.0, 5.0)); // on-axis → screen centre
        let right = pts.iter().find(|p| p.id.as_str() == "right").unwrap();
        // col = 10 + (100/(1000·tan20°))·10 = 12.747477… ; row stays centred (y = 0).
        assert!((right.col - 12.747_477_4).abs() < 1e-5, "got {}", right.col);
        assert_eq!(right.row, 5.0);
    }

    #[test]
    fn on_axis_object_is_centred_regardless_of_area_offset() {
        // The same object projects to the same LOCAL (col, row) wherever the area sits.
        let snap = scene(&[("o", at(0.0, 0.0))]);
        let cam = persp(root(), 1000.0);
        let (p0, _) = project_points(&snap, &cam, Rect::new(0, 0, 20, 10));
        let (p1, _) = project_points(&snap, &cam, Rect::new(7, 3, 20, 10));
        assert_eq!((p0[0].col, p0[0].row), (10.0, 5.0));
        assert_eq!((p1[0].col, p1[0].row), (10.0, 5.0)); // independent of area.x/area.y
    }

    #[test]
    fn camera_relative_state_composes_through_child_frame() {
        // A child frame offset +100 m in x under root; an object +5 m in x within it.
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default())
            .frame(child(), Some(root()), at(100.0, 0.0))
            .object("probe", child(), at(5.0, 0.0), ObjectMeta::default());
        tx.commit();
        let snap = store.snapshot();

        // From the root camera, the probe sits at 105 m in x (frame composition — unchanged).
        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), Rect::new(0, 0, 400, 10));
        let probe = pts.iter().find(|p| p.id.as_str() == "probe").unwrap();
        assert_eq!(probe.pos_cam.x, 105.0);
        // col = 200 + (105/(1000·tan20°))·200 = 257.6970…
        assert!((probe.col - 257.697_026).abs() < 1e-3, "got {}", probe.col);
        assert_eq!(probe.row, 5.0);

        // From a camera riding the child frame, the same probe is only 5 m in x.
        let (pts, _) = project_points(&snap, &persp(child(), 1000.0), Rect::new(0, 0, 40, 10));
        let probe = pts.iter().find(|p| p.id.as_str() == "probe").unwrap();
        assert_eq!(probe.pos_cam.x, 5.0);
        // col = 20 + (5/(1000·tan20°))·20 = 20.274748…
        assert!((probe.col - 20.274_748).abs() < 1e-5, "got {}", probe.col);
        assert_eq!(probe.row, 5.0);
    }

    #[test]
    fn dolly_shrinks_apparent_offset_as_the_eye_pulls_back() {
        // Same off-axis object; a farther dolly makes it converge toward centre (the parallax
        // the log-zoom is about). Near eye: large offset; far eye: ~centred.
        let snap = scene(&[("p", at(100.0, 0.0))]);
        let (near, _) = project_points(&snap, &persp(root(), 1_000.0), area());
        let (far, _) = project_points(&snap, &persp(root(), 1_000_000.0), area());
        let near_off = near[0].col - 10.0;
        let far_off = far[0].col - 10.0;
        assert!((near_off - 2.747_477_4).abs() < 1e-5);
        assert!(
            far_off > 0.0 && far_off < 0.01,
            "far ~centre, got {far_off}"
        );
        assert!(near_off > far_off * 100.0); // ~1000× the offset at 1000× nearer
    }

    #[test]
    fn perspective_parallax_depth_divide() {
        // Two objects at equal x but different depth: the nearer one is more off-centre (the
        // 1/depth divide that distinguishes perspective from the old orthographic map).
        let snap = scene(&[
            ("near", at3(100.0, 0.0, 0.0)),
            ("far", at3(100.0, 0.0, -500.0)),
        ]);
        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), area());
        let near = pts.iter().find(|p| p.id.as_str() == "near").unwrap();
        let far = pts.iter().find(|p| p.id.as_str() == "far").unwrap();
        // near depth = 1000, far depth = 1500 (eye at z=+1000 looking −Z).
        assert!(near.col > far.col && far.col > 10.0);
    }

    #[test]
    fn near_plane_culls_objects_at_or_behind_the_eye() {
        // Eye at z = +1000 looking −Z. An object on the far (+z) side is behind the eye → culled;
        // one in front survives.
        let snap = scene(&[
            ("front", at3(0.0, 0.0, 0.0)),
            ("behind", at3(0.0, 0.0, 2000.0)),
        ]);
        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), area());
        let ids: Vec<&str> = pts.iter().map(|p| p.id.as_str()).collect();
        assert_eq!(ids, ["front"]);
    }

    #[test]
    fn wider_fov_moves_objects_toward_centre() {
        let snap = scene(&[("p", at(100.0, 0.0))]);
        let narrow = persp(root(), 1000.0); // 40°
        let wide = Camera {
            fov: 80f64.to_radians(),
            ..persp(root(), 1000.0)
        };
        let (n, _) = project_points(&snap, &narrow, area());
        let (w, _) = project_points(&snap, &wide, area());
        assert!((n[0].col - 10.0) > (w[0].col - 10.0) && (w[0].col - 10.0) > 0.0);
    }

    #[test]
    fn numerically_safe_across_many_orders_of_magnitude() {
        // Eye dollied 1e11 m (≈ 1 AU) back. A near 50 m object and a 1e9 m **off-axis** object
        // both flow through the perspective divide at ~1e11 m depth → finite, *correct* coords
        // (the off-axis term is non-zero, so this genuinely exercises the large-magnitude divide,
        // not a trivial on-axis centre). Camera-relative coords never difference two huge
        // absolute positions, so no NaN/∞.
        let snap = scene(&[("near", at(50.0, 0.0)), ("far", at(1.0e9, 0.0))]);
        let (pts, _) = project_points(&snap, &persp(root(), 1.0e11), area());
        for p in &pts {
            assert!(p.col.is_finite() && p.row.is_finite());
        }
        // col = 10 + (1e9/(1e11·tan20°))·10 = 10.274747… — the divide is correct at AU scale.
        let far = pts.iter().find(|p| p.id.as_str() == "far").unwrap();
        assert!((far.col - 10.274_747_7).abs() < 1e-4, "got {}", far.col);
        assert!(pts.iter().any(|p| p.id.as_str() == "near"));
    }

    #[test]
    fn tracked_object_target_recentres_on_it() {
        // target = Object("a"); `a` is off-axis but, being the look-at anchor, projects to centre.
        // `b` is off the view axis (offset in z, which becomes screen-right here), so it's
        // off-centre relative to `a`.
        let snap = scene(&[("a", at(100.0, 0.0)), ("b", at3(100.0, 0.0, 50.0))]);
        let cam = Camera {
            target: CameraTarget::Object("a".into()),
            up: UpHint::Direction(DVec3::Y),
            zoom: LogZoom::from_distance(1000.0),
            ..Camera::overview(root())
        };
        let (pts, _) = project_points(&snap, &cam, area());
        let a = pts.iter().find(|p| p.id.as_str() == "a").unwrap();
        assert_eq!((a.col, a.row), (10.0, 5.0)); // the tracked object is centred
        let b = pts.iter().find(|p| p.id.as_str() == "b").unwrap();
        assert!(b.col > 10.0); // the other object is off-centre relative to it
    }

    #[test]
    fn bearing_target_sets_the_view_axis() {
        // target = Bearing(+X): the eye looks along +X; an object along +X is centred.
        let snap = scene(&[("ahead", at(100.0, 0.0))]);
        let cam = Camera {
            target: CameraTarget::Bearing(DVec3::X),
            up: UpHint::Direction(DVec3::Z),
            zoom: LogZoom::from_distance(1000.0),
            ..Camera::overview(root())
        };
        let (pts, _) = project_points(&snap, &cam, area());
        let ahead = pts.iter().find(|p| p.id.as_str() == "ahead").unwrap();
        assert_eq!((ahead.col, ahead.row), (10.0, 5.0));
    }

    // ---- #19: angular-size LOD ----

    // A unit-sphere body at the origin on the root frame; with `persp(root, dist)` its on-screen
    // radius is `r_eq/(dist·tan20°)·(w/2) = 1/(dist·0.363970)·10 = 27.47474/dist` cells. So a
    // target radius S → `dist = 27.47474/S`.
    fn unit_sphere() -> astrodyn_planet::PlanetShape {
        astrodyn_planet::PlanetShape::new("unit", 1.0, 1.0, 1.0, 0.0)
    }
    fn shaped_scene(shape: astrodyn_planet::PlanetShape) -> std::sync::Arc<crate::scene::Snapshot> {
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default()).object(
            "b",
            root(),
            at(0.0, 0.0),
            ObjectMeta {
                shape: Some(BodyShape::ellipsoid(shape)),
                ..ObjectMeta::default()
            },
        );
        tx.commit();
        store.snapshot()
    }
    // `dist` giving on-screen radius `s` for the unit sphere in the 20×10 area.
    fn dist_for(s: f64) -> f64 {
        27.474_74 / s
    }
    fn lod_at(s: f64, mem: &mut LodMemory) -> Lod {
        let (b, _) = project_bodies(
            &shaped_scene(unit_sphere()),
            &persp(root(), dist_for(s)),
            area(),
            mem,
        );
        b[0].lod
    }

    #[test]
    fn lod_first_frame_uses_the_band_midpoint() {
        // No prior → classify against (GROW+SHRINK)/2 = 1.125 cells.
        assert_eq!(lod_at(1.3, &mut LodMemory::new()), Lod::Ellipsoid); // > 1.125
        assert_eq!(lod_at(1.0, &mut LodMemory::new()), Lod::Point); // < 1.125
    }

    #[test]
    fn lod_hysteresis_holds_inside_the_band() {
        // Once an Ellipsoid, a size inside the band (0.75..1.5) does NOT drop back to Point.
        let mut up = LodMemory::new();
        assert_eq!(lod_at(2.0, &mut up), Lod::Ellipsoid); // grow
        assert_eq!(lod_at(1.0, &mut up), Lod::Ellipsoid); // in band → stays Ellipsoid
                                                          // Once a Point, a size inside the band does NOT grow to Ellipsoid.
        let mut down = LodMemory::new();
        assert_eq!(lod_at(0.4, &mut down), Lod::Point); // start small
        assert_eq!(lod_at(1.0, &mut down), Lod::Point); // in band → stays Point
    }

    #[test]
    fn lod_grows_and_shrinks_outside_the_band() {
        let mut m = LodMemory::new();
        assert_eq!(lod_at(0.4, &mut m), Lod::Point);
        assert_eq!(lod_at(1.6, &mut m), Lod::Ellipsoid); // ≥ GROW (1.5)
        assert_eq!(lod_at(0.5, &mut m), Lod::Point); // < SHRINK (0.75)
    }

    #[test]
    fn shapeless_object_is_always_a_point() {
        // A huge, close, shapeless object still resolves to a point (no shape → no angular size).
        let snap = scene(&[("x", at(0.0, 0.0))]);
        let (b, _) = project_bodies(&snap, &persp(root(), 100.0), area(), &mut LodMemory::new());
        assert_eq!(b[0].lod, Lod::Point);
    }

    #[test]
    fn lod_memory_prunes_objects_that_leave_the_scene() {
        let mut m = LodMemory::new();
        assert_eq!(lod_at(2.0, &mut m), Lod::Ellipsoid); // remembers "b" = Ellipsoid
                                                         // A frame with the root frame but no objects prunes "b" (retain_seen drops unseen ids).
        let empty = {
            let store = SceneStore::new();
            let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
            tx.frame(root(), None, BodyState::default());
            tx.commit();
            store.snapshot()
        };
        let _ = project_bodies(&empty, &persp(root(), 10.0), area(), &mut m);
        // "b" reappears at a band size (1.0 < midpoint): if its stale Ellipsoid were kept it would
        // stay Ellipsoid; pruned, it re-seeds from the midpoint → Point.
        assert_eq!(lod_at(1.0, &mut m), Lod::Point);
    }

    #[test]
    fn oblate_silhouette_is_circular_pole_on_and_squashed_equator_on() {
        // Oblate spheroid: r_eq 2, r_pol 1 (flattening 0.5 → e² = 0.75, so minor = r_pol/r_eq = ½).
        let shape =
            BodyShape::ellipsoid(astrodyn_planet::PlanetShape::new("ob", 1.0, 2.0, 1.0, 0.5));
        let view = ViewBasis {
            right: DVec3::X,
            up: DVec3::Y,
            forward: DVec3::NEG_Z,
        };
        let ray = DVec3::NEG_Z; // body straight ahead along the view axis
        let sr = 10.0;

        // Pole-on: body polar axis (att_cam·Z) ∥ the ray → a circle, tilt irrelevant (0).
        let (minor, tilt) = oblate_silhouette(&shape, &DMat3::IDENTITY, &view, ray, sr);
        assert!((minor - sr).abs() < 1e-9);
        assert_eq!(tilt, 0.0);

        // Equator-on: rotate body +Z → camera +X (⟂ the ray) → minor squashed to sr·r_pol/r_eq,
        // and the major axis is ⟂ the (horizontal) projected polar → vertical (π/2).
        let eq = DMat3::from_rotation_y(std::f64::consts::FRAC_PI_2);
        let (minor, tilt) = oblate_silhouette(&shape, &eq, &view, ray, sr);
        assert!((minor - sr * 0.5).abs() < 1e-6, "minor {minor}");
        assert!((tilt - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn one_transform_per_frame_applies_to_all_its_objects() {
        // Two objects share an offset child frame; each picks up that frame's transform.
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default())
            .frame(child(), Some(root()), at(100.0, 0.0))
            .object("nose", child(), at(1.0, 0.0), ObjectMeta::default())
            .object("tail", child(), at(-1.0, 0.0), ObjectMeta::default());
        tx.commit();
        let snap = store.snapshot();

        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), Rect::new(0, 0, 400, 10));
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
        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), area());
        let ids: Vec<&str> = pts.iter().map(|p| p.id.as_str()).collect();
        assert_eq!(ids, ["ok"]); // NaN/∞ objects are dropped, not mapped to (0,0)
    }

    #[test]
    fn offscreen_objects_are_culled() {
        let snap = scene(&[("near", at(0.0, 0.0)), ("far", at(1_000.0, 0.0))]);
        let (pts, _) = project_points(&snap, &persp(root(), 1000.0), area());
        let ids: Vec<&str> = pts.iter().map(|p| p.id.as_str()).collect();
        assert_eq!(ids, ["near"]); // "far" is off the 20-wide viewport
    }

    #[test]
    fn unknown_camera_frame_yields_nothing_but_is_reported() {
        let snap = scene(&[("a", at(0.0, 0.0))]);
        // `absent` is a valid identity that is simply not present in this scene: nothing
        // renders, and that is surfaced (not a silent blank screen).
        let (pts, report) = project_points(&snap, &Camera::overview(absent()), area());
        assert!(pts.is_empty());
        assert_eq!(report.unresolved_camera_frame, Some(absent()));
        assert!(!report.is_clean());
    }

    #[test]
    fn degenerate_inputs_yield_nothing() {
        // A degenerate lens (fov = 0) draws nothing.
        let snap = scene(&[("a", at(0.0, 0.0))]);
        let bad_fov = Camera {
            fov: 0.0,
            ..Camera::overview(root())
        };
        assert!(project_points(&snap, &bad_fov, area()).0.is_empty());
        // An empty scene is not an error.
        let empty = SceneStore::new().snapshot();
        let (pts, report) = project_points(&empty, &Camera::overview(root()), area());
        assert!(pts.is_empty() && report.is_clean());
    }

    #[test]
    fn orphan_object_is_reported_not_silently_dropped() {
        // An object on a frame that isn't in the tree must be surfaced (DESIGN §4.4).
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default()).object(
            "ghost",
            absent(),
            at(0.0, 0.0),
            ObjectMeta::default(),
        );
        tx.commit();
        let (pts, report) = project_points(&store.snapshot(), &Camera::overview(root()), area());
        assert!(pts.is_empty());
        let orphans: Vec<&str> = report.orphan_objects.iter().map(|o| o.as_str()).collect();
        assert_eq!(orphans, ["ghost"]);
    }

    #[test]
    fn frame_with_unresolvable_parent_is_reported_dropped() {
        // `child`'s declared parent (`absent`) is never added to the scene, so `child` can't
        // be placed — it is reported dropped rather than silently vanishing.
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default())
            .frame(child(), Some(absent()), at(1.0, 0.0))
            .object("here", root(), at(0.0, 0.0), ObjectMeta::default());
        tx.commit();
        let (_pts, report) = project_points(&store.snapshot(), &Camera::overview(root()), area());
        assert_eq!(report.dropped_frames, vec![child()]);
    }

    /// Records the body centres handed to a renderer, so we can check what `SpaceView` projected.
    #[derive(Default)]
    struct Recorder(std::cell::RefCell<Vec<(f64, f64)>>);
    impl Renderer for Recorder {
        fn draw_bodies(&self, bodies: &[RenderBody], _area: Rect, _buf: &mut Buffer) {
            self.0
                .borrow_mut()
                .extend(bodies.iter().map(|b| (b.col, b.row)));
        }
    }

    // A throwaway Vehicle marker for the vehicle-parameterized presets (Lvlh/Ned/BodyFrame).
    astrodyn_quantities::define_vehicle!(TestProbe);

    fn approx(a: DVec3, b: DVec3) -> bool {
        a.abs_diff_eq(b, 1e-12)
    }

    #[test]
    fn presets_name_the_expected_frames_with_default_target_and_up() {
        use astrodyn_quantities::{BodyFrame, Lvlh, Ned, PlanetInertial};
        let cases = [
            (Camera::solar_overview(), FrameUid::of::<RootInertial>()),
            (
                Camera::inertial_chase::<Moon>(),
                FrameUid::of::<PlanetInertial<Moon>>(),
            ),
            (
                Camera::body_fixed::<Moon>(),
                FrameUid::of::<PlanetFixed<Moon>>(),
            ),
            (
                Camera::orbit_relative::<TestProbe>(),
                FrameUid::of::<Lvlh<TestProbe>>(),
            ),
            (
                Camera::vehicle_ned::<TestProbe>(),
                FrameUid::of::<Ned<TestProbe>>(),
            ),
            (
                Camera::onboard::<TestProbe>(),
                FrameUid::of::<BodyFrame<TestProbe>>(),
            ),
        ];
        for (cam, uid) in cases {
            assert_eq!(cam.frame, uid, "preset frame uid");
            // Default target is the camera's own frame origin; default up is frame-up.
            assert_eq!(cam.target, CameraTarget::FrameOrigin(uid));
            assert_eq!(cam.up, UpHint::FrameUp);
        }
        // Distinct planets / vehicles yield distinct identities (tag carries the parameter).
        assert_ne!(
            Camera::body_fixed::<Moon>().frame,
            Camera::body_fixed::<Mars>().frame
        );
    }

    #[test]
    fn local_horizon_sites_have_distinct_value_keyed_identities() {
        // The whole point of astrodyn #688/#696: two sites on one planet must NOT collide, and a
        // given (planet, site) is stable (so a producer and the viz converge on one FrameUid).
        let a = Camera::local_horizon::<Moon>("shackleton");
        let b = Camera::local_horizon::<Moon>("malapert");
        assert_ne!(a.frame, b.frame, "two lunar sites must not alias");
        assert_eq!(
            a.frame,
            Camera::local_horizon::<Moon>("shackleton").frame,
            "same (planet, site) is stable across calls"
        );
        // Same site key on a different planet is still distinct.
        assert_ne!(a.frame, Camera::local_horizon::<Mars>("shackleton").frame);
        // It is a Topocentric identity, and a site uid is NOT the planet-only typed uid.
        assert_eq!(a.frame.class, astrodyn_quantities::FrameClass::Topocentric);
        assert_ne!(
            a.frame,
            FrameUid::of::<astrodyn_quantities::Topocentric<Moon>>()
        );
        // Default target/up match the other presets.
        assert_eq!(a.target, CameraTarget::FrameOrigin(a.frame.clone()));
        assert_eq!(a.up, UpHint::FrameUp);
    }

    #[test]
    fn view_basis_matches_projection_convention() {
        // Eye looks down −Z with +Y up → +X right, +Y up — the screen axes the perspective
        // projector ([`project_perspective`]) uses.
        let cam = Camera {
            up: UpHint::Direction(DVec3::Y),
            ..Camera::overview(root())
        };
        let b = cam.view_basis(DVec3::NEG_Z);
        assert!(approx(b.forward, DVec3::NEG_Z));
        assert!(approx(b.right, DVec3::X));
        assert!(approx(b.up, DVec3::Y));
        // Right-handed with the eye down its own −Z: right × up == −forward.
        assert!(approx(b.right.cross(b.up), -b.forward));
    }

    #[test]
    fn default_frame_up_is_z_so_top_down_is_the_gimbal_case() {
        // The default up is the frame's +Z; looking straight down (−Z) is then up ∥ forward —
        // a genuine top-down ambiguity. The guard must still yield an orthonormal basis.
        let b = Camera::overview(root()).view_basis(DVec3::NEG_Z);
        assert_eq!(Camera::overview(root()).up, UpHint::FrameUp);
        assert!(approx(b.forward, DVec3::NEG_Z));
        for v in [b.right, b.up, b.forward] {
            assert!((v.length() - 1.0).abs() < 1e-12);
        }
        assert!(b.right.dot(b.forward).abs() < 1e-12);
        assert!(approx(b.right.cross(b.up), -b.forward));
    }

    #[test]
    fn view_basis_is_orthonormal_for_an_oblique_axis() {
        let cam = Camera {
            up: UpHint::Direction(DVec3::new(0.0, 1.0, 0.2)),
            ..Camera::overview(root())
        };
        let b = cam.view_basis(DVec3::new(1.0, 2.0, -3.0));
        for v in [b.right, b.up, b.forward] {
            assert!((v.length() - 1.0).abs() < 1e-12, "unit length");
        }
        assert!(b.right.dot(b.up).abs() < 1e-12);
        assert!(b.right.dot(b.forward).abs() < 1e-12);
        assert!(b.up.dot(b.forward).abs() < 1e-12);
        assert!(approx(b.right.cross(b.up), -b.forward)); // right-handed
    }

    #[test]
    fn view_basis_gimbal_guard_when_up_parallel_to_forward() {
        // up ∥ forward would collapse the basis; the guard must still return an orthonormal one.
        let cam = Camera {
            up: UpHint::Direction(DVec3::Z),
            ..Camera::overview(root())
        };
        let b = cam.view_basis(DVec3::Z); // forward == up
        assert!(approx(b.forward, DVec3::Z));
        assert!((b.right.length() - 1.0).abs() < 1e-12);
        assert!(b.right.dot(b.forward).abs() < 1e-12, "right ⟂ forward");
        assert!(approx(b.right.cross(b.up), -b.forward)); // still right-handed
    }

    #[test]
    fn view_basis_gimbal_guard_uses_y_alternate_when_forward_along_x() {
        // forward ∥ +X (and up ∥ forward) takes the `f.x.abs() >= 0.9` branch → alternate up Y.
        let cam = Camera {
            up: UpHint::Direction(DVec3::X),
            ..Camera::overview(root())
        };
        let b = cam.view_basis(DVec3::X); // forward == up == +X
        assert!(approx(b.forward, DVec3::X));
        assert!(approx(b.right, DVec3::Z)); // X × Y = Z
        assert!(approx(b.up, DVec3::Y));
        for v in [b.right, b.up, b.forward] {
            assert!((v.length() - 1.0).abs() < 1e-12);
        }
        assert!(approx(b.right.cross(b.up), -b.forward)); // right-handed
    }

    #[test]
    fn view_basis_zero_forward_falls_back_to_minus_z() {
        let cam = Camera {
            up: UpHint::Direction(DVec3::Y),
            ..Camera::overview(root())
        };
        let b = cam.view_basis(DVec3::ZERO);
        assert!(approx(b.forward, DVec3::NEG_Z)); // degenerate axis → frame look axis
        assert!(approx(b.right, DVec3::X));
        assert!(approx(b.up, DVec3::Y));
    }

    #[test]
    fn camera_target_forward_resolution() {
        // Bearing resolves itself (normalized), ignoring the supplied point.
        let bearing = CameraTarget::Bearing(DVec3::new(0.0, 0.0, -5.0));
        assert_eq!(bearing.forward(None), Some(DVec3::NEG_Z));
        // FrameOrigin / Object resolve from the look-at point in camera coords.
        let origin = CameraTarget::FrameOrigin(child());
        assert_eq!(
            origin.forward(Some(DVec3::new(0.0, 10.0, 0.0))),
            Some(DVec3::Y)
        );
        let tracked = CameraTarget::Object("probe".into());
        assert_eq!(
            tracked.forward(Some(DVec3::new(3.0, 0.0, 0.0))),
            Some(DVec3::X)
        );
        // Degenerate look axes (no point, or target coincident with the eye) yield None.
        assert_eq!(origin.forward(None), None);
        assert_eq!(tracked.forward(Some(DVec3::ZERO)), None);
        assert_eq!(CameraTarget::Bearing(DVec3::ZERO).forward(None), None);
    }

    #[test]
    fn space_view_projects_snapshot_to_the_renderer() {
        let store = SceneStore::new();
        let mut tx = store.writer("p").begin(Epoch::from_seconds(0.0));
        tx.frame(root(), None, BodyState::default())
            .object("origin", root(), at(0.0, 0.0), ObjectMeta::default())
            .object("right", root(), at(100.0, 0.0), ObjectMeta::default());
        tx.commit();

        let cam = persp(root(), 1000.0);
        let recorder = Recorder::default();
        let a = area();
        let mut buf = Buffer::empty(a);
        let mut state = ViewState::new(store);
        SpaceView::new(&cam, &recorder).render(a, &mut buf, &mut state);

        let mut got = recorder.0.into_inner();
        got.sort_by(|x, y| x.0.total_cmp(&y.0));
        // Same projection as project_points: origin at centre, "right" off-centre per perspective.
        assert_eq!(got[0], (10.0, 5.0));
        assert!((got[1].0 - 12.747_477_4).abs() < 1e-5 && got[1].1 == 5.0);
    }
}
