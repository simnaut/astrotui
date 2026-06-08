//! The encoding-agnostic transport interface for astrotui wire envelopes (DESIGN.md §4.3).
//!
//! [`WireCodec`] abstracts *how* a [`SceneDocument`](crate::scene_doc::SceneDocument) /
//! [`SceneSeries`](crate::scene_doc::SceneSeries) becomes bytes and back. JSON ([`Json`]) is
//! the only encoding now; the binary path (postcard) for the high-rate socket feed is P3 —
//! adding it is a new `impl WireCodec`, with no change to callers.
//!
//! **The codec is pure transport — it does no validation.** The keyframe handshake (validate
//! before interpreting any number, DESIGN §4.3/§4.4) is the *envelope's* job: callers run
//! `validate()` after `decode` (and `apply_*` re-validates regardless), so a binary-decoded or
//! hand-built document gets the same gate as a JSON one. The single choke point stays the
//! envelope, not the wire format.

use serde::de::DeserializeOwned;
use serde::Serialize;

/// An encoding for astrotui wire envelopes — bytes ⇄ serde value.
///
/// **Not object-safe**: the methods are generic over the payload type, so there is no
/// `&dyn WireCodec`. That is deliberate — consumers are either generic over the codec
/// (`fn ingest<C: WireCodec>(c: &C, …)`) or hold a concrete [`Json`]; nobody needs to store
/// heterogeneous codecs behind a trait object. (Same shape as serde's own `Serializer`.)
pub trait WireCodec {
    /// Encoding/decoding failure for this codec.
    type Error;

    /// Encode a wire value to bytes.
    ///
    /// # Errors
    /// Returns the codec's error if serialization fails.
    fn encode<T: Serialize>(&self, value: &T) -> Result<Vec<u8>, Self::Error>;

    /// Decode a wire value from bytes. The result is **not** validated — run the envelope's
    /// `validate()` afterward (the keyframe handshake).
    ///
    /// # Errors
    /// Returns the codec's error if deserialization fails.
    fn decode<T: DeserializeOwned>(&self, bytes: &[u8]) -> Result<T, Self::Error>;
}

/// JSON encoding over `serde_json` — the human-readable default (DESIGN §4.3, "JSON first").
/// Carries the workspace's `float_roundtrip` feature, so `f64` survives encode→decode
/// bit-exact (astrodyn_frame_doc RFS-601).
#[derive(Debug, Default, Clone, Copy)]
pub struct Json;

impl WireCodec for Json {
    type Error = serde_json::Error;

    fn encode<T: Serialize>(&self, value: &T) -> Result<Vec<u8>, Self::Error> {
        serde_json::to_vec(value)
    }

    fn decode<T: DeserializeOwned>(&self, bytes: &[u8]) -> Result<T, Self::Error> {
        serde_json::from_slice(bytes)
    }
}
