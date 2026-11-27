#include <Energy/Strand/AdaptedFrame.h>

namespace sim {

auto ComputeEdges(const Mat<Real> &vertices) -> std::vector<Edge> {
  std::vector<Edge> edges;
  edges.reserve(vertices.rows() - 1);
  for (int ii = 0; ii < vertices.rows() - 1; ++ii) {
    edges.emplace_back(vertices.row(ii), vertices.row(ii + 1));
  }
  return edges;
}

void ParallelTransportModifiedBishopFrames(
    const std::vector<Vec3<Real>> &curvatureBinormals,
    const std::vector<Edge> &edges, std::vector<BishopFrame> &frames) {
  SIM_ASSERT(!curvatureBinormals.empty(), "No curvature binormals provided");
  SIM_ASSERT(!edges.empty(), "No edges provided");
  SIM_ASSERT(!frames.empty(), "At least the initial frame is required");

  for (int ii = 1; ii < std::ssize(edges); ++ii) {
    // Get the normalized tangent vector
    Vec3<Real> ti = edges.at(ii).t;

    // We define the rotation matrix as the rotation around the curvature
    // binormal
    const Mat3<Real> rotation =
        RotationMatrixAroundNormal(curvatureBinormals.at(ii - 1));

    // Transport the last frame up with the rotation
    // Iteratively define ui = Pi(ui-1)
    Vec3<Real> u = rotation * frames.at(ii - 1).u;
    u.normalize();

    const Vec3<Real> v = ti.cross(u);

    // Insert the frame
    frames.at(ii) = BishopFrame(ti, u, v);
  }
}

} // namespace sim
