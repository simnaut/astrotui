//! astrotui-dem — DEM ingest / tiling / LOD.
//!
//! Turns lat/lon/height tiles into Cartesian meshes in a body-fixed frame for the
//! rendering core to project. Pipeline lands in P2 and is gated by the DEM design doc
//! (#41): static pre-tiled site → dynamic tiling/paging → LOD + memory budget →
//! multi-backend hillshade.
