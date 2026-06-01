# astrotui — repository guide

A [ratatui](https://github.com/ratatui/ratatui) widget for visualizing space/mission
scenarios in the terminal. **Viz only**: it renders states something else mutates; it
does no propagation, force models, or body placement.

**The design doc [`docs/DESIGN.md`](docs/DESIGN.md) is the source of truth.** When code
and design disagree, the design wins (or the design is updated deliberately, in its own
change). Cite the relevant `§` when implementing.

## The architectural firewall (load-bearing — do not violate)

`astrotui-core` and every `astrotui-render-*` crate link **no Bevy** and **no
ANISE/ephemeris**. Those live only in *producers* that are external to the render core:
the sim-side exporter (the planned `apps/refsim`, P3) and the ephemeris body-filler. The substrate is
the pure astrodyn crates only — `astrodyn_frames`, `astrodyn_quantities`,
`astrodyn_planet` — pinned by git rev in `[workspace.dependencies]`. CI's `firewall` job
enforces this on every PR.

## Conventions

- **Crate layout** follows `docs/DESIGN.md` §9. Keep core source-agnostic; producers and
  hosts live under `apps/` (and may depend on Bevy/ANISE behind a feature / separate CI
  lane, never as a dependency of core).
- **States are kept in their native frames** (glam `DVec3`/`DQuat`), never pre-projected;
  the camera transform is applied per-render. Epoch is `astrodyn_quantities::SecondsSince<TDB>`.
- **Frames/objects carry stable string ids + labels + kind**, so the UI can enumerate them.
- **Tests**: prefer golden-frame snapshots (`insta` + `astrotui_core::testing::buffer_to_text`)
  for rendering, frame-transform checks against known states, and wire round-trips. Tests
  must genuinely exercise behavior — never assert trivially-true things to turn a job green.
- **Edition 2021, MSRV pinned via `rust-version`.** `#![deny(unsafe_code)]` is the
  workspace default; justify any `#[allow(unsafe_code)]`.

## Build & merge gate

```sh
cargo fmt --all --check   # CI runs --check; drop --check locally to auto-format
cargo clippy --all-targets --workspace -- -D warnings
cargo test --workspace
```

`main` is squash-only, takes no direct pushes, and requires three checks: **test**
(fmt/clippy/build/test), **firewall** (core links no Bevy/ANISE), and **claude-review**
(this automated review). Inline review findings block merge until resolved.

## What a rigorous review must check

When reviewing a PR (CI or local), scrutinize — and leave an inline comment for anything
blocking:

1. **Correctness** — bugs, edge cases, error handling, and especially **concurrency**:
   lock poisoning, races, ordering, tear-free reads, `Send`/`Sync` soundness.
2. **Design fidelity** — matches `docs/DESIGN.md` and the locked decisions; flag drift.
3. **Firewall** — no Bevy/ANISE/ephemeris reaches `astrotui-core` or a render crate.
4. **Test adequacy** — tests are present, meaningful, and actually exercise the change;
   call out green-washed or missing tests (concurrency/edge paths especially).
5. **Quality** — no shims, hacks, or dead `TODO`s without a tracking issue; no needless
   complexity; idiomatic Rust; naming and comment density matching surrounding code.
6. **Scope** — one issue per PR, a reviewable diff; flag unrelated changes.
