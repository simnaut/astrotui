# astrotui

A [ratatui](https://github.com/ratatui/ratatui) widget for visualizing space
scenarios in the terminal — anything from an Earth→Jupiter cruise down to a Moon
landing over real DEM terrain. Inspired by
[`d10n/tui-globe`](https://github.com/d10n/tui-globe), but for whole scenes, not
a single globe.

**Status:** concept / pre-implementation.

## The idea

A camera *is* a reference frame, so switching cameras collapses to one call —
`compute_relative_state` against a different frame — using the
[`astrodyn`](https://github.com/simnaut/astrodyn) flight-dynamics workspace as its
substrate. astrotui is **viz only**: it renders states that something else
mutates. The core links neither Bevy nor ANISE; data arrives over a
self-describing wire stream, so a live sim, a replay log, and real telemetry all
look the same to the widget.

See **[docs/DESIGN.md](docs/DESIGN.md)** for the full architecture (diagrams
included).
