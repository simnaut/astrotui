//! astrotui-core — the source-agnostic rendering core.
//!
//! Holds the widget, [`Camera`](crate), the `SceneStore`/`SceneWriter` ingestion
//! model, the `Renderer` trait, and the projection/LOD pipeline. Per `docs/DESIGN.md`
//! (§2, §12) this crate links **no Bevy** and **no ANISE/ephemeris** — it renders the
//! states it is given. Implementation lands in P0 and later.

pub mod render;
pub mod scene;

/// Test-only helpers (golden-frame snapshots). Available in this crate's tests and,
/// for downstream crates, via the `testing` feature.
#[cfg(any(test, feature = "testing"))]
pub mod testing;

#[cfg(test)]
mod astrodyn_surface {
    //! Compile/link verification that the pinned astrodyn substrate (#8, post-#645)
    //! exposes the surface astrotui-core relies on. This is a resolution check — if the
    //! pin drifts and the API moves, this fails to compile rather than failing silently.
    use astrodyn_frames::{FrameId, FrameTree, RefFrameState};
    use astrodyn_planet::{PlanetShape, MOON};
    use astrodyn_quantities::{CartesianState, Position, RootInertial, Velocity};

    // Type-level references — fail to compile if any of these types are gone/renamed.
    #[allow(dead_code)]
    fn _types(
        _tree: &FrameTree,
        _id: FrameId,
        _state: RefFrameState,
        _shape: PlanetShape,
        _cs: CartesianState<RootInertial>,
        _p: Position<RootInertial>,
        _v: Velocity<RootInertial>,
    ) {
    }

    #[test]
    fn surface_resolves() {
        // The dynamic relative-state primitive the render loop is built on (DESIGN.md §3):
        // bind it as a fn pointer so the exact signature is checked without a FrameTree.
        let _relative_state: fn(&FrameTree, FrameId, FrameId) -> RefFrameState =
            FrameTree::compute_relative_state;

        // PlanetShape presets + accessors (r_eq/r_pol are private fields, read via methods).
        // `black_box` keeps clippy from folding these into constant assertions.
        use std::hint::black_box;
        assert_eq!(black_box(MOON.name), "Moon");
        assert!(black_box(MOON.r_eq()) >= black_box(MOON.r_pol()));
        assert!(black_box(MOON.mu) > 0.0);
    }
}

#[cfg(test)]
mod snapshot_harness {
    //! Seeds the golden-frame snapshot mechanism (insta + `buffer_to_text`) that
    //! P0+ render tests build on.
    use crate::testing::buffer_to_text;
    use ratatui::{buffer::Buffer, layout::Rect, style::Style};

    #[test]
    fn buffer_dump_snapshots() {
        let mut buf = Buffer::empty(Rect::new(0, 0, 6, 3));
        buf.set_string(0, 0, "astro", Style::default());
        buf.set_string(1, 2, "tui", Style::default());
        insta::assert_snapshot!(buffer_to_text(&buf));
    }
}
