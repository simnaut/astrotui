//! A time-driven **replay player** over a [`SceneSeries`] (DESIGN.md §8(b)).
//!
//! [`Replay`] decodes + validates a recorded scene series **once**, flattens its
//! segment/epoch grid into a play-ordered timeline, and lets a host drive a [`SceneWriter`]
//! **at controlled sim time**: ask for the frame at-or-before a time `t` and it applies that
//! epoch (frames + objects together) via [`apply_scene_series_epoch`]. Per-epoch apply
//! re-validates only the targeted row (#21), so stepping a whole recording is `O(total rows)`.
//!
//! The host owns the clock: each render tick it maps elapsed time to a sim time and calls
//! [`Replay::apply_at`]. A sim EOF is just "no later cue" — the last snapshot stays on screen,
//! the viz lives on (§4).

use std::io;
use std::path::Path;

use astrotui_core::scene::SceneWriter;

use crate::codec::{Json, WireCodec};
use crate::frame_doc::ApplyError;
use crate::scene_doc::{apply_scene_series_epoch, SceneSeries};

/// One applicable epoch in the flattened replay timeline.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Cue {
    segment: usize,
    epoch: usize,
    simtime: f64,
}

/// A decoded, validated scene replay positioned by sim time.
#[derive(Debug, Clone)]
pub struct Replay {
    series: SceneSeries,
    /// Cues in play order; `simtime` is non-decreasing (a recording is time-ordered).
    timeline: Vec<Cue>,
}

/// Failure loading a [`Replay`] from bytes/file.
#[derive(Debug)]
pub enum ReplayError {
    /// Reading the file failed.
    Io(io::Error),
    /// Decoding the bytes failed (JSON).
    Decode(serde_json::Error),
    /// The decoded series failed validation (the keyframe handshake / congruence).
    Invalid(ApplyError),
}

impl std::fmt::Display for ReplayError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReplayError::Io(e) => write!(f, "reading replay file: {e}"),
            ReplayError::Decode(e) => write!(f, "decoding replay: {e}"),
            ReplayError::Invalid(e) => write!(f, "invalid replay: {e}"),
        }
    }
}

impl std::error::Error for ReplayError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ReplayError::Io(e) => Some(e),
            ReplayError::Decode(e) => Some(e),
            ReplayError::Invalid(e) => Some(e),
        }
    }
}

impl Replay {
    /// Build a player from an already-decoded [`SceneSeries`], validating it once (whole-series:
    /// frame handshake + timeline congruence) and flattening its timeline.
    ///
    /// # Errors
    /// [`ApplyError`] if the series fails validation.
    pub fn new(series: SceneSeries) -> Result<Self, ApplyError> {
        series.validate()?;
        let mut timeline = Vec::new();
        for (segment, seg) in series.frames.segments.iter().enumerate() {
            for (epoch, row) in seg.epochs.iter().enumerate() {
                timeline.push(Cue {
                    segment,
                    epoch,
                    simtime: row.simtime,
                });
            }
        }
        Ok(Self { series, timeline })
    }

    /// Decode a JSON-encoded [`SceneSeries`] and build a player (decode → `new`).
    ///
    /// # Errors
    /// [`ReplayError::Decode`] on malformed JSON, [`ReplayError::Invalid`] on a series that
    /// fails validation.
    pub fn from_json_slice(bytes: &[u8]) -> Result<Self, ReplayError> {
        let series: SceneSeries = Json.decode(bytes).map_err(ReplayError::Decode)?;
        Self::new(series).map_err(ReplayError::Invalid)
    }

    /// Read and decode a recorded replay file (JSON).
    ///
    /// # Errors
    /// [`ReplayError::Io`] if the file can't be read, else as [`Replay::from_json_slice`].
    pub fn from_json_path(path: impl AsRef<Path>) -> Result<Self, ReplayError> {
        let bytes = std::fs::read(path).map_err(ReplayError::Io)?;
        Self::from_json_slice(&bytes)
    }

    /// Number of applicable epochs (cues) in the timeline.
    #[must_use]
    pub fn len(&self) -> usize {
        self.timeline.len()
    }

    /// `true` if the replay has no epochs.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.timeline.is_empty()
    }

    /// The `(first, last)` cue sim times, or `None` if empty.
    #[must_use]
    pub fn time_bounds(&self) -> Option<(f64, f64)> {
        match (self.timeline.first(), self.timeline.last()) {
            (Some(a), Some(b)) => Some((a.simtime, b.simtime)),
            _ => None,
        }
    }

    /// The decoded series (e.g. to inspect its uid table / header).
    #[must_use]
    pub fn series(&self) -> &SceneSeries {
        &self.series
    }

    /// Index of the cue to show at sim time `t`: the **last** cue with `simtime <= t`, or `None`
    /// if `t` precedes the first cue. Relies on the timeline being time-ordered (a recording is).
    #[must_use]
    pub fn cue_at_time(&self, t: f64) -> Option<usize> {
        // partition_point: count of cues with simtime <= t; one before that is the last such.
        self.timeline
            .partition_point(|c| c.simtime <= t)
            .checked_sub(1)
    }

    /// Apply the cue at `index` (its frames + objects) to `w`.
    ///
    /// # Errors
    /// [`ApplyError::NoSuchEpoch`] if `index` is out of range, else as
    /// [`apply_scene_series_epoch`].
    pub fn apply_cue(&self, index: usize, w: &mut SceneWriter) -> Result<(), ApplyError> {
        let cue = self.timeline.get(index).ok_or(ApplyError::NoSuchEpoch)?;
        apply_scene_series_epoch(&self.series, cue.segment, cue.epoch, w)
    }

    /// Apply the frame **at-or-before** sim time `t`. Returns the applied cue index, or `None`
    /// (nothing applied — leave the prior snapshot on screen) if `t` precedes the first cue.
    ///
    /// # Errors
    /// As [`apply_scene_series_epoch`] for the resolved epoch.
    pub fn apply_at(&self, t: f64, w: &mut SceneWriter) -> Result<Option<usize>, ApplyError> {
        match self.cue_at_time(t) {
            Some(i) => {
                self.apply_cue(i, w)?;
                Ok(Some(i))
            }
            None => Ok(None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use astrodyn_frame_doc::{
        CanonicalRotation, Conventions, DocHeader, FrameRecord, Origin, SeriesBuilder, TransRecord,
        SCHEMA_VERSION,
    };
    use astrodyn_quantities::{FrameUid, Moon, PlanetFixed, RootInertial};
    use astrotui_core::scene::SceneStore;
    use glam::DVec3;

    use crate::scene_doc::{ObjectEpochRow, ObjectKindWire, ObjectRecord, ObjectSegment};

    fn header() -> DocHeader {
        DocHeader {
            schema_version: SCHEMA_VERSION,
            conventions: Conventions::current(),
            simtime: 0.0,
            tai_tjt_at_epoch: 0.0,
        }
    }
    fn root_uid() -> FrameUid {
        FrameUid::of::<RootInertial>()
    }
    fn child_uid() -> FrameUid {
        FrameUid::of::<PlanetFixed<Moon>>()
    }
    fn ident() -> CanonicalRotation {
        CanonicalRotation::Quat([1.0, 0.0, 0.0, 0.0])
    }
    fn frec(uid_index: u32, parent: Option<u32>, simtime: f64, x: f64) -> FrameRecord {
        FrameRecord {
            name: "f".into(),
            uid_index,
            parent,
            epoch: Some(simtime),
            trans: TransRecord {
                position: [x, 0.0, 0.0],
                velocity: [0.0; 3],
            },
            rotation: ident(),
            ang_vel_this: [0.0; 3],
            origin: Origin::Injected,
        }
    }
    fn obj(x: f64) -> ObjectRecord {
        ObjectRecord {
            id: "probe".into(),
            label: "probe".into(),
            frame_index: 1,
            kind: ObjectKindWire::Spacecraft,
            trans: TransRecord {
                position: [x, 0.0, 0.0],
                velocity: [0.0; 3],
            },
            rotation: ident(),
            shape: None,
            path: None,
        }
    }
    // A 3-epoch, single-segment series at simtimes 0,1,2; child + probe advance in x.
    fn series() -> SceneSeries {
        let mut b = SeriesBuilder::new(header(), vec![root_uid(), child_uid()]);
        for (t, x) in [(0.0, 10.0), (1.0, 20.0), (2.0, 30.0)] {
            b.push_epoch(t, vec![frec(0, None, t, 0.0), frec(1, Some(0), t, x)]);
        }
        let frames = b.finish();
        let objects = vec![ObjectSegment {
            start_simtime: 0.0,
            epochs: (0..3)
                .map(|i| ObjectEpochRow {
                    simtime: i as f64,
                    objects: vec![obj((i + 1) as f64)], // 1,2,3
                })
                .collect(),
        }];
        SceneSeries { frames, objects }
    }

    #[test]
    fn timeline_and_bounds() {
        let r = Replay::new(series()).unwrap();
        assert_eq!(r.len(), 3);
        assert!(!r.is_empty());
        assert_eq!(r.time_bounds(), Some((0.0, 2.0)));
    }

    #[test]
    fn cue_at_time_picks_last_at_or_before() {
        let r = Replay::new(series()).unwrap();
        assert_eq!(r.cue_at_time(-0.5), None); // before the first cue
        assert_eq!(r.cue_at_time(0.0), Some(0)); // exact
        assert_eq!(r.cue_at_time(0.9), Some(0)); // between → previous
        assert_eq!(r.cue_at_time(1.0), Some(1));
        assert_eq!(r.cue_at_time(99.0), Some(2)); // past the end → last
    }

    #[test]
    fn apply_at_drives_the_writer_to_the_right_epoch() {
        let r = Replay::new(series()).unwrap();
        let store = SceneStore::new();

        // Before the start: nothing applied, store stays empty.
        assert_eq!(r.apply_at(-1.0, &mut store.writer("replay")).unwrap(), None);
        assert!(store.snapshot().is_empty());

        // At t=1.5 → cue 1: child at 20, probe at 2.
        assert_eq!(
            r.apply_at(1.5, &mut store.writer("replay")).unwrap(),
            Some(1)
        );
        let snap = store.snapshot();
        let probe = snap
            .objects()
            .iter()
            .find(|o| o.id.as_str() == "probe")
            .unwrap();
        assert_eq!(probe.state.position, DVec3::new(2.0, 0.0, 0.0));
        let child = snap.frames().iter().find(|f| f.uid == child_uid()).unwrap();
        assert_eq!(child.state.position, DVec3::new(20.0, 0.0, 0.0));

        // Advance to t=2 → cue 2: probe at 3.
        assert_eq!(
            r.apply_at(2.0, &mut store.writer("replay")).unwrap(),
            Some(2)
        );
        let probe = store
            .snapshot()
            .objects()
            .iter()
            .find(|o| o.id.as_str() == "probe")
            .unwrap()
            .clone();
        assert_eq!(probe.state.position, DVec3::new(3.0, 0.0, 0.0));
    }

    #[test]
    fn json_round_trip_via_from_slice() {
        let bytes = Json.encode(&series()).unwrap();
        let r = Replay::from_json_slice(&bytes).unwrap();
        assert_eq!(r.len(), 3);
        assert_eq!(r.time_bounds(), Some((0.0, 2.0)));
    }

    #[test]
    fn from_slice_rejects_invalid_series() {
        // Make the object timeline incongruent with the frames → validation fails on load.
        let mut s = series();
        s.objects[0].epochs.pop();
        let bytes = Json.encode(&s).unwrap();
        let err = Replay::from_json_slice(&bytes).unwrap_err();
        assert!(matches!(err, ReplayError::Invalid(_)));
    }

    #[test]
    fn from_json_path_reads_a_file() {
        let bytes = Json.encode(&series()).unwrap();
        let path =
            std::env::temp_dir().join(format!("astrotui_replay_{}.json", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let r = Replay::from_json_path(&path).unwrap();
        assert_eq!(r.len(), 3);
        let _ = std::fs::remove_file(&path);

        let missing = std::env::temp_dir().join("astrotui_replay_does_not_exist_xyz.json");
        assert!(matches!(
            Replay::from_json_path(missing).unwrap_err(),
            ReplayError::Io(_)
        ));
    }
}
