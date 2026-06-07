# astrotui — Design

> A [ratatui](https://github.com/ratatui/ratatui) widget for visualizing space
> scenarios in the terminal — anything from an Earth→Jupiter cruise down to a
> Moon landing over real DEM terrain. Inspired by
> [`d10n/tui-globe`](https://github.com/d10n/tui-globe), but for whole scenes,
> not a single globe.

**Status:** concept / pre-implementation. This document is the anchor; code
follows it.

---

## 1. Goals & non-goals

### Goals

- Render **any scene in the solar system** at any scale — interplanetary cruise,
  planetary system, orbit, surface approach, touchdown — with a single,
  seamless camera model.
- **Intuitive cameras** the user switches between (inertial chase, body-fixed,
  orbit-relative, local horizon, onboard), all defined in terms the user already
  thinks in: *reference frames*.
- Be a **drop-in plugin** for any host that wants space/mission viz in a TUI.
  The primary host is a long-lived **simulation orchestrator** that can watch a
  running sim, replay a log, or listen to a live telemetry/ops stream — but
  astrotui knows nothing about its host.
- For the Moon-landing case, render **realistic surface shape from DEMs**, since
  terrain drives where a lander can set down.

### Non-goals

- **No trajectory propagation.** astrotui is *viz only*: it renders the states it
  is handed. Something else (a sim, or real telemetry) owns and mutates the
  states.
- **No orbit determination, no force models, no time integration.**
- **No body placement.** astrotui does not compute where the Moon is; bodies
  arrive in the data stream like any other object. astrotui-core therefore links
  **neither Bevy nor ANISE/ephemeris** — it is a pure renderer.
- DEM *acquisition* and tiling pipeline detail is owned here, but the dynamics,
  frames, ephemeris, and body shapes are **not reimplemented** — they come from
  the [`astrodyn`](https://github.com/simnaut/astrodyn) workspace.

---

## 2. Substrate: the astrodyn workspace

astrotui is built on astrodyn (an ECS-based, NASA-JEOD-derived flight-dynamics
workspace in Rust). We consume its reference frames, typed quantities,
transforms, and body shapes rather than rolling our own — but **astrotui-core
depends only on the pure, Bevy-free, ANISE-free astrodyn crates**
(`astrodyn_frames`, `astrodyn_quantities`, `astrodyn_planet`). Bevy and ephemeris
appear only in *producers* that live outside astrotui (§4).

| Need | astrodyn surface | Used by |
|---|---|---|
| Frame tree + relative state | `astrodyn_frames`: `FrameTree`, `compute_relative_state(from, to) -> RefFrameState`; typed `orchestration::compute_relative_state_typed::<From, To>()` | **core** (render pass) |
| Typed frames | `astrodyn_quantities::frame`: `RootInertial`, `PlanetInertial<P>`, `PlanetFixed<P>`, `Ecef`, `BodyFrame<V>`, `Lvlh<C>`, `Ned<C>`, `Topocentric<P>`; planets `Earth/Moon/Sun/Mars`; `define_planet!`/`define_vehicle!` | **core** + producers |
| Runtime frame identity *(astrodyn #659)* | `astrodyn_quantities::frame_descriptor`: `FrameUid { namespace, class, role, tag }`, `Frame::DESCRIPTOR`, `FrameUid::of::<F>()` / `is::<F>()` / `external(…)` | **core** (type, no serde) + wire + producers |
| Typed quantities | `Position<F>` / `Velocity<F>` = `Qty3<D, F>` (`#[repr(C)]` over 3×f64), `raw_si() -> DVec3`, `from_raw_si()`, `zero()` | **core** + wire |
| Rotations | `NormalizedQuat<ScalarFirst, LeftTransform>` (JEOD convention), `FrameTransform<From, To>` (`.apply()`, `.matrix()`, `.inverse()`, `Mul` composition) | **core** |
| Frame wire format *(astrodyn #659)* | `astrodyn_frame_doc`: `FrameDocument` (snapshot) + `FrameSeries` (replay) and their record types — `DocHeader`/`Conventions`/`FrameRecord`/`EpochRow`/`CanonicalRotation`/`Origin` (bit-exact, serde; deps = `astrodyn_quantities` + serde only, never the physics crates) | **wire** (frames) |
| Body shape | `astrodyn_planet::PlanetShape { mu, r_eq, r_pol, flat_coeff }`; presets `EARTH/MOON/SUN/MARS/JUPITER/SATURN` | **core** (LOD/ellipsoid/DEM) |
| Body positions *(producer only)* | `astrodyn_ephemeris`: `Ephemeris::get_state_typed(target, observer, tdb) -> (Position<RootInertial>, Velocity<RootInertial>)` (DE421/DE440 via ANISE) | ephemeris body-filler (orchestrator-side) |
| Body-fixed orientation *(producer only)* | `Ephemeris::get_body_rotation_to::<P>(body, BodyFixedFrame, epoch) -> FrameTransform<RootInertial, PlanetFixed<P>>` (`Iau`, `MoonPaDe421`, `MoonMeDe421`) | sim exporter / body-filler |
| ECS state *(producer only)* | `astrodyn_bevy`: `TranslationalStateC<P>`, `RotationalStateC`, frame entities, `FrameOrigin`/`RelativeFrameState`, `SimulationTimeR` | sim-side exporter |

> All of the orientation/ME/IAU/serde/Topocentric surface above landed via
> astrodyn issue [#645](https://github.com/simnaut/astrodyn/issues/645), filed
> from this project. Before #645, the Moon-ME path required reaching around
> astrodyn into ANISE directly; it no longer does — and that path now lives in a
> *producer*, never in astrotui-core.

**What astrotui owns:** all rendering (projection, rasterization, terminal
backends), the scene data model + ingestion API, DEM ingest + tiling + LOD, the
self-describing wire codec, and body-shape presets beyond the six above (the rest
are one-line `PlanetShape::new`). astrodyn has no combined "scene" concept — that
is this project.

---

## 3. Core concept: a camera *is* a reference frame

Because the plugin only renders provided states, the entire per-object pipeline
collapses to **one astrodyn call** — *evaluate relative state against the camera's
frame*:

```rust
// Where is `obj` as seen from the camera's frame? Frames are named by FrameUid;
// the tree resolves each to a node, then composes the relative state.
let s = compute_relative_state(&tree, cam.frame, obj.frame);  // by resolved node
let pos_cam: DVec3 = s.trans.position.raw_si();   // metres, in camera coordinates
let vel_cam: DVec3 = s.trans.velocity.raw_si();   // m/s   -> leading indicators / HUD
let att_cam: DMat3 = s.rot.t_parent_this();        // object body axes, in camera frame
// -> log-depth project pos_cam -> braille / color cell / sixel
```

Everything an object needs to be drawn — position, orientation, velocity — comes
back in the **camera's own coordinates**. Therefore:

- **Switching cameras = evaluating relative state against a different frame.**
  No bespoke camera math per mode; the frame tree already encodes every
  relationship.
- The user's mental model ("look at this from the Moon-fixed frame", "ride the
  lander", "watch from the orbit-relative frame") maps 1:1 onto astrodyn's
  *existing* frame markers.

> **Identity is `astrodyn_quantities::FrameUid`** *(revised 2026-06-07 — adopts
> astrodyn [#659](https://github.com/simnaut/astrodyn/issues/659); supersedes the
> 2026-06-04 home-grown `FrameKind` enum + `Other(String)` plan, which is no longer
> built).* astrodyn now ships a first-class runtime frame identity; astrotui
> **consumes it** rather than reinventing one:
>
> ```rust
> // astrodyn_quantities::frame_descriptor
> struct FrameUid { namespace: Namespace, class: FrameClass, role: FrameRole, tag: Tag }
> ```
>
> This *is* the "closed vocabulary + escape hatch" we were going to hand-roll, done
> upstream and better:
> - **Closed vocabulary**: `FrameClass` × `FrameRole` cover the supported frames.
> - **Escape hatch**: `FrameRole::Custom` + `FrameUid::external(non-LOCAL namespace …)`
>   represent producer-invented frames with **no `impl Frame`** required — the
>   runtime-only tail, namespaced so it can't collide with astrodyn identities.
> - **Category *and* identity**: `tag` carries the parameter (e.g. `PlanetFixed`
>   with `tag = Moon`), so the earlier "category-not-identity" ceiling is gone —
>   `FrameUid` distinguishes Moon-fixed from Mars-fixed, not just "a planet-fixed".
> - **Typed ⇄ runtime bridge**: every sealed marker carries `Frame::DESCRIPTOR`, so
>   `FrameUid::of::<F>()` lowers a compile-time frame to its uid and
>   `uid.is::<F>()` recovers/checks it — lossless, at fixed compile-time sites.
>
> Two boundaries to keep straight:
> - **Core holds `FrameUid` without serde.** The type lives in `astrodyn_quantities`
>   (already a core dep); its `serde` derives are feature-gated and needed only by
>   the wire crate, so `astrotui-core` adopts `FrameUid` as its identity and stays
>   serde-free. The firewall is untouched (`astrodyn_quantities` is Bevy/ANISE-free).
> - **Render resolves by node.** Geometry is still evaluated via
>   `compute_relative_state(from, to)` over a `FrameTree`; `FrameUid` is the stable
>   *identity/handle* objects and the camera name their frame by, and the tree
>   resolves a uid to a node (`FrameTree::resolve(&uid)`, with a loud miss). Identity
>   is never a substitute for the node the per-vertex math walks.

Camera presets — each is just a `FrameUid` the camera sits in:

| Preset | astrodyn frame | Scenario |
|---|---|---|
| Solar-system overview | `RootInertial` | Earth→Jupiter cruise |
| Inertial chase | `PlanetInertial<P>` | orbits (non-rotating) |
| Body-fixed | `PlanetFixed<P>` | ground track, lunar approach (any body via `BodyFixedFrame::Iau`) |
| Orbit-relative | `Lvlh<V>` | nadir / ram-pointed from a spacecraft |
| Local horizon | `Topocentric<P>` | landing site / ground station, anchored at lat/lon |
| Vehicle local | `Ned<V>` | a *moving* vehicle's local NED |
| Onboard | `BodyFrame<V>` | cockpit / sensor boresight |

Seamless **log-zoom** layers cleanly on top: the *frame* sets origin and
orientation; a scalar **log-distance** dollies the eye along the view axis;
angular-size **LOD** picks point → shaded ellipsoid → DEM mesh as a body grows
on screen. The frame and the zoom are orthogonal.

---

## 4. System architecture

The decisive fact: **the viz outlives the things it watches.** A simulation runs
and terminates; a telemetry session connects and drops; the orchestrator (and the
astrotui widget inside it) persists across all of them. There is no shared
memory to borrow — data crosses a **lifecycle/process boundary by value.**

```mermaid
flowchart TB
  subgraph ORCH["Orchestrator app — long-lived (owns viz, camera, sim lifecycle)"]
    direction TB
    subgraph CORE["astrotui-core — no Bevy, no ANISE"]
      STORE[("SceneStore<br/>scoped layers · snapshot / double-buffer")]
      VIEW["SpaceView widget + Camera"]
      STORE -- "latest snapshot (lock-free read)" --> VIEW
      VIEW -- render --> FRAME["ratatui Frame"]
    end
    SIMR["wire reader · layer 'sim'"]
    TELR["wire reader · layer 'telemetry'"]
    EPHF["ephemeris body-filler · layer 'bodies'<br/>(astrodyn_ephemeris)"]
    SIMR -- "SceneWriter.commit()" --> STORE
    TELR -- "SceneWriter.commit()" --> STORE
    EPHF -- "SceneWriter.commit()" --> STORE
  end

  subgraph SIM["astrodyn_bevy sim — separate process, ephemeral"]
    EXP["exporter system<br/>serialize scene → socket"]
  end
  OPS[/"telemetry / ops feed<br/>(any sim or real ops)"/]

  EXP == "self-describing wire stream (IPC)" ==> SIMR
  OPS == "self-describing wire stream" ==> TELR

  classDef ext fill:#f6f6f6,stroke:#999,stroke-dasharray:4 3;
  class SIM,OPS,EXP,EPHF ext;
```

> Dashed/grey nodes are **external to astrotui** — the sim's exporter ships with
> the sim; the ephemeris body-filler is an orchestrator-side producer. Both speak
> only `astrotui-wire` / `SceneWriter`; neither is a dependency of the render core.

The viz outliving the sim is the load-bearing lifecycle property — shown over time:

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Orch as Orchestrator (+ astrotui)
    participant Sim as Sim process (ephemeral)
    participant Ops as Telemetry feed

    User->>Orch: configure + launch sim
    Orch->>Sim: spawn process
    loop while alive
        Sim-->>Orch: wire stream (header, then samples)
        Note over Orch: commit() into "sim" layer → render latest
    end
    Sim--xOrch: terminate (stream EOF)
    Note over Orch: last snapshot HELD on screen — viz lives on
    User->>Orch: attach telemetry
    Ops-->>Orch: wire stream (different socket)
    Note over Orch: same reader → "telemetry" layer,<br/>"bodies" layer fills reference bodies
```

- **Sims run as separate processes.** The orchestrator spawns one and reads the
  **self-describing wire stream** (§4.3) off a socket. A sim crash is a stream
  EOF, not a viz crash. The last snapshot stays on screen; the viz lives.
- **Telemetry is the same reader on a different socket** — a live sim and real
  ops data are indistinguishable to astrotui.
- **Bevy and ephemeris never enter astrotui.** The sim-side **exporter** (a small
  `astrodyn_bevy` system that serializes the wire format) ships *with the sim*.
  An **ephemeris body-filler** is an orchestrator-side producer that supplies
  reference bodies for feeds that don't carry them.

### 4.1 The plugin API — how data gets in

The host owns one `SceneStore` for the app's whole life. Producers receive
`SceneWriter` handles and publish into **named, isolated layers**:

```rust
// host owns the store for the app's lifetime:
let mut scene = SceneStore::new();
let camera    = Camera::overview();      // host-owned, user-driven

// each producer gets an isolated, named layer (Send + Clone handle):
let sim    = scene.writer("sim");        // fast spacecraft + frames from the sim stream
let bodies = scene.writer("ephemeris");  // slow, low-cadence reference bodies

// a producer publishes a transaction; commit() atomically replaces ITS layer:
let mut tx = sim.begin(epoch);
tx.frame (frame_id, parent_id, rel_state);   // frame node + state relative to parent
tx.object(obj_id, frame_id, state, meta);    // object state in its NATIVE frame
tx.commit();                                  // back-buffer swap; visible next render

// each TUI tick the host renders the latest committed snapshot — the union of layers:
frame.render_stateful_widget(SpaceView::new(&camera), area, &mut scene);
```

**Scoped layers** are the multi-producer model. Each writer owns a disjoint set
of frame/object ids; `commit()` replaces only that layer; the rendered scene is
the live **union** of layers. This gives every producer independent **lifecycle**
*and* **cadence** for free: the `"sim"` layer streams fast and freezes at EOF, the
`"ephemeris"` layer refreshes lazily (bodies barely move at real-time scale), and
a `"telemetry"` layer can take over when a sim ends — all without clobbering each
other.

```mermaid
flowchart LR
  P1["producer: 'sim'<br/>fast cadence"] --> L1["layer 'sim'<br/>lander, frames"]
  P2["producer: 'bodies'<br/>slow cadence"] --> L2["layer 'bodies'<br/>Earth, Moon, Sun"]
  P3["producer: 'telemetry'<br/>on attach"] --> L3["layer 'telemetry'<br/>spacecraft"]
  L1 --> U(("live union<br/>= rendered scene"))
  L2 --> U
  L3 --> U
  U --> R["SpaceView render"]
```

> **In-process `Producer` first; the frame wire is `astrodyn_frame_doc`** *(revised
> 2026-06-07).* The seam producers plug into is an in-process trait —
> `trait Producer { fn populate(&self, w: &mut SceneWriter); }` — and the
> orchestrator demo's body placement is its first impl. The frame wire is **no
> longer home-grown**: a streaming `Producer` consumes `astrodyn_frame_doc` record
> types off a socket (§4.3), honoring its keyframe handshake. Building the socket
> transport waits for a real second producer; `SceneWriter` / layers / snapshot are
> the durable seam. (The multi-process picture in §4's diagram remains the
> *eventual* P3 architecture — this defers *when* it's built, not *whether*.)

### 4.2 The scene data model

astrotui-core **owns** the canonical render model (the `SceneStore`); producers
**populate it by value**. (Borrowing a producer's storage is impossible here —
the producer is across a process/lifecycle boundary — so the copy is inherent to
*observing* an external source, not avoidable waste.)

States are kept **in their natural frames**, *not* pre-projected into the camera
frame. The camera transform is applied per **render**, not per **ingest**, so
camera switching and log-zoom stay smooth at terminal frame rates even when a
producer is slow or bursty.

```rust
pub struct SceneObject {
    pub id:    ObjectId,
    pub label: Cow<'static, str>,        // shown in the camera/frame switcher UI
    pub frame: FrameUid,                 // identity of the FrameTree node this object lives in (astrodyn #659)
    pub kind:  ObjectKind,               // Body | Spacecraft | Site | Marker
    pub shape: Option<BodyShape>,        // astrodyn_planet::PlanetShape (+ optional DEM handle)
    pub trail: TrailRef,                 // plugin-PROVIDED ring buffer, PRODUCER-populated
    pub path:  Option<PathRef>,          // producer-supplied planned polyline
}
```

Frames and objects carry **stable ids + human labels + kind**, so the host's
camera UI can enumerate `scene.frames()` / `scene.objects()` and let the user
target any of them (the in-TUI frame/camera switcher, §10/P4).

**`FrameUid` is both the handle and the classification.** An object (and the
camera) names its frame by its `FrameUid` — the `FrameTree` resolves a uid to a
node (`resolve(&uid)`, loud on miss, §4.4), so the uid is what binds objects to
frames and what dangling-detection tests. Because `FrameUid` carries
`class`/`role`/`tag` intrinsically (§3), there is **no separate `kind` field** —
identity and classification are one value. Core's `FrameRecord` is the in-memory
dual of `astrodyn_frame_doc`'s wire record (§4.3), with uid-table indices resolved
to `FrameUid`s and the frame's state held natively:

```rust
pub struct FrameRecord {
    pub uid:    FrameUid,           // identity (carries class/role/tag); astrodyn #659
    pub parent: Option<FrameUid>,  // None == root
    pub epoch:  Option<Epoch>,     // per-frame time-validity (astrodyn_frame_doc RFS-603)
    pub state:  BodyState,         // trans + canonical rotation + ang-vel, relative to parent
}
```

**Concurrency.** A producer commits on its own thread into a back buffer; the
swap is atomic; the widget reads the latest snapshot lock-free. Trails are
append-only ring buffers; a snapshot pins the current write index so the render
sees a consistent prefix without copying the buffer.

### 4.3 The wire format — one self-describing stream

The **frame** wire is **`astrodyn_frame_doc`** *(astrodyn #659 — supersedes the
home-grown self-describing codec previously specified here).* astrodyn ships a
physics-free schema crate (deps = `astrodyn_quantities` + serde only) whose record
types are the supported public surface; astrotui consumes them directly. The same
types serve socket, telemetry, and replay:

- **Snapshot** = `FrameDocument { header, uids, records }`.
- **Replay** = `FrameSeries { header, uids, segments → EpochRow → records }`.
- **Identity** is interned: a `FrameRecord` names its frame and parent by `u32`
  index into the document's `Vec<FrameUid>`; each record also carries `epoch`,
  `TransRecord`, `CanonicalRotation` (quat **or** matrix, whichever was canonical
  at the write site — bit-exact), `ang_vel`, and an `Origin` (integrated / derived
  / injected). `DocHeader` carries `schema_version` + in-band `Conventions`.

**Live-feed handshake** (the contract astrodyn specified for streaming the record
types over a socket, rather than loading a whole-document JSON blob):

1. **Keyframe = `DocHeader` + the interned uid table, sent once.** Then stream
   per-epoch `EpochRow`s. Use a binary serde format (e.g. postcard) for the
   high-rate path — binary f64 is inherently bit-exact, so the JSON-only
   `float_roundtrip` caveat doesn't apply.
2. **Validate before interpreting any number.** Check `schema_version` +
   `Conventions` at the handshake (the loose-row equivalents of the document's
   `validate_header` / `validate_uid_table`, since `FrameSeries::validate`'s
   whole-document invariants don't run on a socket stream).
3. **Per-record `parent` is self-checking.** Verify each record's parent against
   the folded topology; a mismatch is **loud inconsistency, never reinterpretation**
   (§4.4) — this is the RFS-301/302 transplant guard against the stale-parent
   ~10⁵ km failure mode.
4. **Topology change → re-send a keyframe** (the v1 segment rule: close the
   segment, re-send header + uid table). astrodyn's planned **v2** streaming
   topology events (`Reparent` / `AddFrame` / `RemoveFrame`, additive under
   `schema_version`) would make a frame switch a first-class renderable event;
   not needed for the P2 static-topology slice, available on request.
5. **Pin the `astrodyn_frame_doc` version on both ends and gate on
   `schema_version`** at handshake; pre-1.0 the schema evolves additively
   (`Origin` / `CanonicalRotation` may gain variants behind a version bump).

Core holds `FrameUid` without serde (§3); the serde-bearing `astrodyn_frame_doc`
lives in the wire/adapter crate, so the render core stays serde-free. The
**object/scene layer is astrotui's** — `astrodyn_frame_doc` models *frames only*,
so objects (with `kind`/`shape`/`trail`/`path`) ride alongside in astrotui's own
record, referencing frames by `FrameUid`.

### 4.4 The render pass

```rust
pub struct Camera {
    pub frame:  FrameUid,      // the eye sits in / is oriented by this frame (astrodyn #659)
    pub target: CameraTarget,  // a frame origin, a tracked object, or a fixed bearing
    pub zoom:   LogZoom,       // log-distance dolly along the view axis
    pub up:     UpHint,
    pub fov:    f64,
}
```

Per frame, over the latest snapshot's `FrameTree`:

1. **Bucket** objects (and their trails/paths) by native frame.
2. **Resolve once:** for each *occupied* frame `F`, compute a single
   `compute_relative_state(camera.frame, F)` and cache the transform.
3. **Apply** the cached transform by matrix-vector to every object/trail/path
   vertex in `F` → `pos_cam`/`vel_cam`/`att_cam`. (A planned path of thousands of
   points costs one tree-walk, not thousands.)
4. **Project** `pos_cam` through view + log-depth.
5. **LOD** on angular size → point | shaded ellipsoid | DEM mesh.
6. **Rasterize** into the active backend's cell buffer → ratatui `Buffer`.

> **Unresolved frames and stale parents are loud** *(revised 2026-06-07).* An
> object or record whose frame doesn't `resolve(&uid)` in the snapshot's
> `FrameTree`, **or** whose per-record `parent` disagrees with the folded topology
> (§4.3), is **surfaced** — a logged warning plus a visible orphan marker / status
> line — never silently culled or reinterpreted. This is the RFS-301/302 transplant
> guard (a stale parent is the ~10⁵ km failure mode), and it replaces the P0
> skeleton's silent `else continue`.
>
> **Orientation correctness is an integration test against `astrodyn_frame_doc`,
> gating P2.** astrodyn now carries rotation as first-class data (`CanonicalRotation`,
> bit-exact) and the runner already stamps real `PlanetFixed<…>` identities, so the
> earlier hand-rolled typed→wire→untyped proof is unnecessary. The remaining check:
> deserialize a `FrameDocument`/`FrameSeries` containing a rotating frame, resolve
> it through the `FrameTree`, run `compute_relative_state`, and assert `att_cam` /
> `pos_cam` against a hand-computed reference (plus the loud parent-mismatch path).
> That integration test **gates the P2 epic**.

```mermaid
flowchart LR
  SNAP["latest snapshot<br/>(FrameTree + objects)"] --> BKT["bucket by<br/>native frame"]
  BKT --> RES["resolve ONCE per occupied frame<br/>compute_relative_state(cam.frame, F)"]
  RES --> APP["apply transform · matrix×vec<br/>objects + trail/path vertices"]
  APP --> PRJ["project<br/>view + log-depth"]
  PRJ --> LOD{"angular-size<br/>LOD"}
  LOD -->|small| PT["point"]
  LOD -->|grows| EL["shaded ellipsoid"]
  LOD -->|large| MESH["DEM mesh"]
  PT --> RAS["rasterize → backend cells → ratatui Buffer"]
  EL --> RAS
  MESH --> RAS
```

Working in camera-relative coordinates is also what keeps the projection
numerically safe across ~12 orders of magnitude: the render never differences two
huge absolute positions — magnitudes near the camera are small by construction.

---

## 5. Rendering

### 5.1 Pluggable backends

A `Renderer` trait with three backends, auto-detected from terminal
capabilities (highest available wins, braille is the floor):

| Backend | Technique | DEM fidelity |
|---|---|---|
| Braille | monochrome braille dots (tui-globe style) | wireframe / contour silhouette |
| Color cells | 24-bit color half-blocks, dithered shading | shaded heightfield (hypsometric/hillshade) |
| Graphics protocol | sixel / kitty / iTerm raster | near-photoreal hillshade |

### 5.2 Scale: one seamless log-zoom camera

A single logarithmic-depth camera spans the ~12 orders of magnitude between
interplanetary cruise and a lunar touchdown. Angular-size LOD on
`pos_cam.length()` vs `shape.r_eq()` selects the representation, with hysteresis
on the thresholds so bodies near a boundary don't oscillate:

```mermaid
flowchart LR
  P["point"] -->|grows| E["shaded ellipsoid"] -->|grows| M["DEM mesh"]
  M -.->|shrinks| E -.->|shrinks| P
```

The dashed back-edges denote the hysteresis: the grow and shrink thresholds
differ, so a body hovering at a boundary doesn't flip representations every
frame.

---

## 6. Moon-landing path (DEM)

This is the first target scenario. Post-#645 the *frame* integration is the
lowest-risk part; the **DEM pipeline is the real new engineering** (§9, §11).

1. **Body-fixed frame, supplied by the producer.** Lunar cartography (LOLA,
   SLDEM2015) is referenced to the **Mean-Earth/mean-rotation (ME)** frame,
   NAIF 31007 — *not* the principal-axis frame the gravity field uses. The sim
   exporter (or, for an ops feed, the ephemeris body-filler) computes the ME
   transform and ships it as a frame in the stream:

   ```rust
   // PRODUCER side (sim exporter or body-filler) — not astrotui-core:
   let me: FrameTransform<RootInertial, PlanetFixed<Moon>> =
       eph.get_body_rotation_to::<Moon>(EphemerisBody::Moon, BodyFixedFrame::MoonMeDe421, epoch)?;
   tx.frame(moon_fixed_id, root_id, me_as_rel_state);
   ```

   astrodyn bundles the PA→ME frame kernel (`moon_fk_de421.epa`) and applies the
   offset internally; no parallel ANISE Almanac, no hardcoded constant. astrotui
   just receives the resulting frame.

2. **DEM → geometry (astrotui-dem).** DEM tiles (lat/lon/height over the lunar
   mean radius) → Cartesian in `PlanetFixed<Moon>` using
   `astrodyn_planet::MOON` (r_eq 1738.14 km, r_pol 1736.07 km) + height. Those
   vertices live on the Moon-fixed node, so `compute_relative_state` to the camera
   renders them with no special-casing.

3. **Camera.** `Topocentric<Moon>` anchored at the target site (a child node under
   the Moon-fixed frame, declared by the producer) for the local-horizon descent
   view; `PlanetFixed<Moon>` for the overhead approach.

4. **Lighting.** The Sun is an ordinary object in the stream; its position
   relative to the Moon-fixed frame (via `compute_relative_state`) gives the
   hillshade direction. No ephemeris in astrotui.

5. **Overlays.** Producer-populated descent **trail** + producer-supplied planned
   **path**; flight-path marker from `vel_cam`.

---

## 7. Trajectories

The plugin never propagates. Two complementary overlays, both producer-driven:

- **Rolling trail** — astrotui *provides* the ring-buffer container; the
  **producer populates it** at full rate (the store appends on each committed
  upsert, so no telemetry sample is dropped between renders). Telemetry-friendly.
- **Planned path** — the producer supplies a complete polyline for the
  planned/future track. Sim-friendly.

Both are stored in their natural frame and projected through the active camera
like any other geometry.

---

## 8. Reference / harness app (`orchestrator`)

One binary proves the *same widget* against the *same wire reader* on three
sources, with the viz outliving each:

- **(a) Live sim** — spawn an astrodyn Bevy lunar-descent sim as a child process;
  read its exported wire stream.
- **(b) Replay** — read a recorded descent file (same wire format) and play it
  back.
- **(c) Telemetry** — connect to a socket carrying a spacecraft-only feed; the
  ephemeris body-filler layer supplies Earth/Moon/Sun.

The sim terminates; the orchestrator stays up, last frame held, ready to attach
the next source. Moon-landing-first anchors development; a later Earth→Jupiter
cruise scene proves the broad "any solar-system scene" claim (Jupiter/Saturn from
presets, any body via `BodyFixedFrame::Iau`).

---

## 9. Crate layout

```
astrotui/
├── crates/
│   ├── astrotui-core            # widget + Renderer trait + Camera + SceneStore/SceneWriter
│   │                            #   deps: ratatui, astrodyn_frames, astrodyn_quantities, astrodyn_planet
│   │                            #   NO bevy, NO ephemeris/ANISE
│   ├── astrotui-render-braille  # braille backend
│   ├── astrotui-render-color    # 24-bit color-cell backend
│   ├── astrotui-render-graphics # sixel/kitty backend
│   ├── astrotui-wire            # self-describing stream codec (socket / replay); enables `serde`
│   └── astrotui-dem             # DEM ingest / tiling / LOD
└── apps/
    └── orchestrator             # reference host: spawn-sim + replay + telemetry listen

  external to astrotui (not in this workspace):
    • sim-side exporter   — astrodyn_bevy system that serializes the wire format (ships with the sim)
    • ephemeris body-filler — orchestrator-side producer using astrodyn_ephemeris
```

`astrotui-core` depends only on the pure, Bevy-free, ephemeris-free astrodyn
crates. Bevy enters only in the sim's exporter; ANISE/ephemeris only in the
body-filler producer. Both speak `astrotui-wire` / `SceneWriter` — neither is a
dependency of the render core.

```mermaid
flowchart TD
  subgraph WS["astrotui workspace"]
    CORE["astrotui-core"]
    BR["astrotui-render-braille"]
    CL["astrotui-render-color"]
    GR["astrotui-render-graphics"]
    WIRE["astrotui-wire"]
    DEM["astrotui-dem"]
    ORCH["apps/orchestrator"]
  end

  RAT["ratatui"]
  AF["astrodyn_frames"]
  AQ["astrodyn_quantities"]
  AP["astrodyn_planet"]
  AB["astrodyn_bevy + Bevy"]
  AE["astrodyn_ephemeris + ANISE"]

  EXP["sim-side exporter"]
  FILL["ephemeris body-filler"]

  CORE --> RAT & AF & AQ & AP
  BR & CL & GR --> CORE
  WIRE --> AQ
  DEM --> AP
  ORCH --> CORE & BR & CL & GR & WIRE & DEM & FILL

  EXP --> AB & WIRE
  FILL --> AE & WIRE

  classDef ext fill:#f6f6f6,stroke:#999,stroke-dasharray:4 3;
  class AB,AE,EXP,FILL ext;
  classDef heavy stroke:#c33,stroke-width:2px;
  class AB,AE heavy;
```

> The red-outlined nodes (`Bevy`, `ANISE`) are the heavy / churn-prone deps —
> note they connect **only** to the dashed external producers, never to
> `astrotui-core` or any render crate. That firewall is the whole point.

---

## 10. Phasing

| Phase | Deliverable |
|---|---|
| **P0** | `SceneStore` + scoped `SceneWriter` + snapshot/double-buffer; braille backend; `RootInertial` overview; a trivial in-process producer feeding Earth/Moon/Sun points. *Validates camera=frame + projection + ingestion.* |
| **P1** | Camera presets + seamless log-zoom + LOD (point → shaded ellipsoid); color-cell backend. *(Wire codec + replay reader **deferred** behind the in-process `Producer` trait — §4.1.)* |
| **Pre-P2 hardening** *(added 2026-06-04; rescoped 2026-06-07 for astrodyn #659)* | Adopt `astrodyn_quantities::FrameUid` as the core frame identity (§3); consume `astrodyn_frame_doc` record types for the frame wire (§4.3); introduce the in-process `Producer` trait (§4.1); make unresolved frames + stale parents loud (§4.4); and the **integration test consuming a rotating-frame `FrameDocument` that gates P2** (§4.4). |
| **P2** | Moon-landing slice: producer-supplied `MoonMeDe421` frame, `Topocentric<Moon>` camera, trail + path. **Blocked by the Pre-P2 rotating-frame integration test (§4.4).** **DEM via a dedicated design doc + staged build:** (1) one static pre-tiled site → mesh → shade end-to-end, (2) dynamic tiling/paging, (3) LOD + memory budget, (4) hillshade across all backends. |
| **P3** | Separate-process sim + exporter; orchestrator spawn/observe lifecycle; telemetry listen + ephemeris body-filler layer; backend auto-detect. |
| **P4** | Earth→Jupiter cruise scene, HUD, in-TUI frame/camera switcher driven by `scene.frames()`. |

---

## 11. Open items

- **DEM design doc** (LOLA vs SLDEM2015, tile scheme, on-disk vs fetched, LOD,
  memory budget) — owned by `astrotui-dem`, gates P2 code.
- **Backpressure/coalescing policy** — when a producer outruns the TUI frame
  rate, ingestion processes every sample (so trails stay complete) while render
  reads the latest; the firehose fallback (coalesce, accept trail gaps) is a
  tunable to specify in P3.
- **Backend capability detection** heuristics (sixel/kitty probing, graceful
  fallback) — to be specified in P3.
- **Four planet presets** (Mercury/Venus/Uranus/Neptune) self-supplied via
  `PlanetShape::new` until astrodyn adds them (#645 added Jupiter/Saturn only;
  bodies are oblate-only, no triaxial).

---

## 12. Key decisions (summary)

1. **Camera = reference frame**; switching cameras = `compute_relative_state`
   against a different frame. Frame identity is **`astrodyn_quantities::FrameUid`**
   (§3, adopts astrodyn #659; supersedes the 2026-06-04 home-grown `FrameKind`
   plan) — namespace/class/role/tag, with `FrameUid::of`/`is` as the typed bridge
   and `external` + `FrameRole::Custom` for runtime-only frames. Transform resolved
   once per occupied frame, applied by matrix to all geometry.
2. **astrotui is a host-agnostic plugin**; the primary host is a long-lived
   orchestrator whose **viz lifecycle is independent of the sim's**.
3. **Sims run as separate processes** streaming a wire format; **telemetry is the
   same reader on a different socket**. Sim EOF ≠ viz death; last snapshot holds.
4. **astrotui-core links no Bevy and no ANISE/ephemeris** — a pure renderer.
   Bevy lives only in the sim-side exporter; ephemeris only in an
   orchestrator-side body-filler. **Bodies arrive in the stream** like any object.
5. **Core owns the `SceneStore`; producers populate it by value** (the
   lifecycle/process boundary makes the copy inherent, not waste).
6. **Ingestion is scoped layers**: named `SceneWriter`s own disjoint id sets;
   `commit()` replaces only that layer; render = live union; independent lifecycle
   *and* cadence per producer.
7. **Frame interchange is `astrodyn_frame_doc`** (astrodyn #659): a keyframe
   (`DocHeader` + interned `FrameUid` table) then per-epoch `EpochRow` records —
   bit-exact, validated, replayable without the producer's code. `astrotui-wire` is
   the **outer framing** (the socket keyframe handshake + the object/scene records
   that ride alongside, referencing frames by `FrameUid`); it serves socket,
   telemetry, and replay identically.
8. **Snapshot / double-buffer** decouples producer rate from TUI frame rate;
   states kept in natural frames, camera applied per-render.
9. **Trails are a plugin-provided container, producer-populated** at full rate;
   planned paths are producer-supplied. Never propagated here.
10. **Pluggable backends** (braille / color / graphics), auto-detected.
11. **One seamless log-zoom camera** with angular-size LOD (hysteresis on
    transitions).
12. **DEM is the real P2 lift** — full pipeline, but gated by a dedicated design
    doc and built in verifiable stages.
