#include <Energy/Strand/DiscreteElasticRod.h>

DiscreteElasticRod::DiscreteElasticRod(Mat<Real> vertices)
    : vertices(std::move(vertices)), nRods(vertices.rows() - 1) {
  // At least 3 nodes is required
  ASSERT(this->vertices.rows() >= 3, "At least 3 nodes is required");

  // initial position of centerline in rest state
  restVertices = this->vertices;
  Initialize();
}
void DiscreteElasticRod::Initialize() {
  // initial velocity of centerline
  velocities = Vec<Real>::Zero(vertices.rows());

  // Thetas are per edge
  thetas = Vec<Real>::Zero(nRods);

  // Lengths are per edge
  lengths = Vec<Real>::Zero(nRods);
  restLengths = Vec<Real>::Zero(nRods);

  for (int ii = 0; ii < nRods; ++ii) {
    const Real length = (vertices.row(ii + 1) - vertices.row(ii)).norm();
    lengths(ii) = length;
    restLengths(ii) = length;
  }

  // free, clamped or body-coupled ends
  // assume that the conditions are always clamped on one
  // TODO

  // Make initial frames
  Frame initialFrame;
  // Get the normalized tangent vector
  initialFrame.t = vertices.row(1) - vertices.row(0);
  initialFrame.t.normalize();

  // Get u0 as a random orthogonal vector
  initialFrame.u = OrthogonalVector(initialFrame.t);

  // v0 is the cross product of t0 and u0
  initialFrame.v = initialFrame.t.cross(initialFrame.u);

  // Insert the frame
  frames.emplace_back(initialFrame);

  // Compute the curvature binormals
  Computekbs();

  // Compute the bishop frames with the roated parallel transport
  UpdateBishopFrames();
}

void DiscreteElasticRod::UpdateBishopFrames() {
  for (int ii = 1; ii < nRods; ++ii) {
    // Get the normalized tangent vector
    Vec3<Real> ti = vertices.row(ii + 1) - vertices.row(ii);
    ti.normalize();

    // Get an orthogonal vector between ti and ti-1
    const Vec3<Real> n = ti.cross(frames.at(ii - 1).t);

    // Compute the rotation matrix with the vector n
    const Mat3<Real> rotation = RotationMatrixAroundNormal(n);

    // Transport the last frame up with the rotation
    Vec3<Real> u = rotation * frames.at(ii - 1).u;
    u.normalize();

    const Vec3<Real> v = ti.cross(u);

    // Insert the frame
    frames.emplace_back(ti, u, v);
  }
}

void DiscreteElasticRod::Computekbs() {
  kbs.clear();
  for (int ii = 0; ii < nRods - 1; ++ii) {
    // Get the edges on either side of the vertex
    const Vec3<Real> e0 = vertices.row(ii + 1) - vertices.row(ii);
    const Vec3<Real> e1 = vertices.row(ii + 2) - vertices.row(ii + 1);

    // Discrete curvature binormal
    kbs.emplace_back(2.0 * e0.cross(e1) /
                     (restLengths(ii) * restLengths(ii + 1) + e0.dot(e1)));
  }
}
