//! The in-process producer seam (DESIGN.md §4.1).
//!
//! A [`Producer`] publishes scene content into one layer of a
//! [`SceneStore`](crate::scene::SceneStore) through a layer-scoped [`SceneWriter`]. This is
//! the durable in-process boundary DESIGN §4.1 mandates: the orchestrator demo and the
//! `astrotui-wire` frame-document consumer are the first impls, and a streaming socket reader
//! (P3) is just another `Producer`.

use crate::scene::SceneWriter;

/// Something that publishes scene content into one layer via a [`SceneWriter`].
///
/// The host hands the producer a writer already scoped to a named layer; the producer owns
/// its whole publish cycle — `w.begin(epoch)` → stage frames/objects → `commit` — because the
/// epoch and cadence are producer-specific. The trait is object-safe, so a host can drive a
/// `&dyn Producer` (or a list of them).
pub trait Producer {
    /// Populate the scene through `w`: stage a transaction and commit it.
    fn populate(&self, w: &mut SceneWriter);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{BodyState, Epoch, SceneStore};
    use astrodyn_quantities::{FrameUid, RootInertial};

    struct OneFrame;
    impl Producer for OneFrame {
        fn populate(&self, w: &mut SceneWriter) {
            let mut tx = w.begin(Epoch::from_seconds(0.0));
            tx.frame(FrameUid::of::<RootInertial>(), None, BodyState::default());
            tx.commit();
        }
    }

    #[test]
    fn producer_populates_through_a_writer() {
        let store = SceneStore::new();
        OneFrame.populate(&mut store.writer("p"));
        assert_eq!(store.snapshot().frames().len(), 1);
    }

    #[test]
    fn producer_is_object_safe() {
        let store = SceneStore::new();
        let p: &dyn Producer = &OneFrame;
        p.populate(&mut store.writer("p"));
        assert_eq!(store.snapshot().frames().len(), 1);
    }
}
