//! Small geometry helpers used by the render pass.

use glam::DVec3;

/// The centroid (mean position) of a set of points, in their shared frame.
pub fn centroid(points: &[DVec3]) -> DVec3 {
    let sum = points.iter().copied().fold(DVec3::ZERO, |acc, p| acc + p);
    sum / points.len() as f64
}
