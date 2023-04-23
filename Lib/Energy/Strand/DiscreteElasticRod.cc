#include <Energy/Strand/DiscreteElasticRod.h>

DiscreteElasticRod::DiscreteElasticRod(Mat<Real> vertices)
    : vertices(std::move(vertices)), nRods(vertices.rows() - 1) {
  // At least 3 nodes is required
  ASSERT(this->vertices.rows() >= 3, "At least 3 nodes is required");

  // initial position of centerline in rest state
  restVertices = this->vertices;

  // Initialize the mass matrix - uniform mass right now.
  Vec<Real> masses = Vec<Real>::Ones(DOFs());
  mInv = ConstructDiagonalSparseMatrix(masses);

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

  // Material frames are per edge
  m1s.resize(nRods);
  m2s.resize(nRods);

  for (int ii = 0; ii < nRods; ++ii) {
    const Real length = (vertices.row(ii + 1) - vertices.row(ii)).norm();
    lengths(ii) = length;
    restLengths(ii) = length;
  }

  // free, clamped or body-coupled ends
  // assume that the conditions are always clamped on one end
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
  frames.resize(nRods);
  frames.at(0) = initialFrame;

  // Compute the curvature binormals
  Computekbs();

  // Compute the bishop frames with the rotated parallel transport
  UpdateBishopFrames();

  // Compute the updated material frames before computing curvature
  UpdateMaterialFrames();

  // Get the curvature update
  UpdateMaterialCurvatures();

  // Set rest curvatures
  restPrevCurvature = prevCurvature;
  restNextCurvature = nextCurvature;

  // Update the material curvature gradients
  UpdateKbGradients();

  // Update material holonomy
  UpdateHolonomyGradient();
}

auto DiscreteElasticRod::ComputeCenterlineForces() -> Vec<Real> {
  Vec<Real> R = Vec<Real>::Zero(DOFs());

  // J is a 90 degree rotation matrix. Always
  Mat2<Real> J;
  J << 0, -1, 1, 0;

  // Compute the derivative of the curvature
  const Mat2<Real> Bhat = Mat2<Real>::Identity() * BENDING_MODULUS;

  // Formula in 7.1, the General case
  for (int ii = 1; ii < vertices.rows(); ++ii) {
    Vec3<Real> vertexForce = Vec3<Real>::Zero();

    for (int jj = ii - 1; jj <= ii; ++jj) {
      const Mat2x3<Real> gradW = ComputeGradW(
          kbs.at(jj), gradientkbs.at(jj), m1s.at(jj), m2s.at(jj),
          holonomy.at(jj),
          jj == ii - 1 ? prevCurvature.at(jj) : nextCurvature.at(jj));

      const Vec2<Real> w =
          jj == ii - 1 ? prevCurvature.at(jj) : nextCurvature.at(jj);
      const Vec2<Real> wbar =
          ii - 1 ? restPrevCurvature.at(jj) : restNextCurvature.at(jj);

      vertexForce += gradW.transpose() * Bhat * (w - wbar);
    }

    vertexForce /= restLengths(ii - 1);

    R.segment<3>(3 * ii - 1) = vertexForce;
  }

  return R;
}

void DiscreteElasticRod::UpdateBishopFrames() {
  ASSERT2(!kbs.empty());
  for (int ii = 1; ii < nRods; ++ii) {
    // Get the normalized tangent vector
    Vec3<Real> ti = vertices.row(ii + 1) - vertices.row(ii);
    ti.normalize();

    // We define the rotation matrix as the rotation around the curvature
    // binormal
    const Mat3<Real> rotation = RotationMatrixAroundNormal(kbs.at(ii - 1));

    // Transport the last frame up with the rotation
    // Iteratively define ui = Pi(ui-1)
    Vec3<Real> u = rotation * frames.at(ii - 1).u;
    u.normalize();

    const Vec3<Real> v = ti.cross(u);

    // Insert the frame
    frames.at(ii) = Frame(ti, u, v);
  }
}

void DiscreteElasticRod::UpdateMaterialFrames() {
  for (int ii = 0; ii < nRods; ++ii) {
    const Real theta = thetas(ii);
    const Real cosTheta = std::cos(theta);
    const Real sinTheta = std::sin(theta);
    const Vec3<Real> m1 =
        cosTheta * frames.at(ii).u + sinTheta * frames.at(ii).v;
    const Vec3<Real> m2 =
        -sinTheta * frames.at(ii).u + cosTheta * frames.at(ii).v;
    m1s.at(ii) = m1;
    m2s.at(ii) = m2;
  }
}

void DiscreteElasticRod::UpdateMaterialCurvatures() {
  nextCurvature.clear();
  prevCurvature.clear();

  for (int ii = 1; ii < kbs.size(); ++ii) {
    for (int jj = ii - 1; jj <= ii; ++jj) {
      const Vec3<Real> kb = kbs.at(jj);
      const Vec3<Real> m1 = m1s.at(jj);
      const Vec3<Real> m2 = m2s.at(jj);

      if (jj == ii - 1) {
        prevCurvature.emplace_back(ComputeW(kb, m1, m2));
      } else {
        nextCurvature.emplace_back(ComputeW(kb, m1, m2));
      }
    }
  }
}

void DiscreteElasticRod::UpdateKbGradients() {
  gradientLhskbs.resize(nRods);
  gradientRhskbs.resize(nRods);
  gradientkbs.resize(nRods);

  // Here [e] is a skew-symmetric 3 × 3 matrix acting on 3-vectors x by [e] · x
  // = e × x.
  for (int ii = 0; ii < nRods - 1; ++ii) {
    // Lhs
    const Vec3<Real> e0 = vertices.row(ii + 1) - vertices.row(ii);
    const Mat3<Real> crossLhs = CrossProductMatrix(e0);

    // Rhs
    const Vec3<Real> e1 = vertices.row(ii + 2) - vertices.row(ii + 1);
    const Mat3<Real> crossRhs = CrossProductMatrix(e1);

    // Compute the lhs gradient
    const Real denominator = restLengths(ii) * restLengths(ii + 1) + e0.dot(e1);

    // DEBUG: This could be a broken thing. The paper just _had_ to switch the
    // damn terms.
    gradientLhskbs.at(ii) =
        2 * crossRhs + kbs.at(ii) * e1.transpose() / denominator;
    gradientRhskbs.at(ii) =
        2 * crossLhs + kbs.at(ii) * e0.transpose() / denominator;
  }
}

void DiscreteElasticRod::UpdateHolonomyGradient() {
  holonomy.resize(nRods);
  holonomyLhs.resize(nRods);
  holonomyRhs.resize(nRods);

  for (int ii = 0; ii < nRods - 1; ++ii) {
    holonomyLhs.at(ii) = kbs.at(ii) / 2 * (restLengths(ii));
    holonomyRhs.at(ii) = -kbs.at(ii) / 2 * (restLengths(ii + 1));
    holonomy.at(ii) = -1 * (holonomyLhs.at(ii) + holonomyRhs.at(ii));
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

auto DiscreteElasticRod::ComputeW(const Vec3<Real> &kb, const Vec3<Real> &m1,
                                  const Vec3<Real> &m2) -> Vec2<Real> {
  return {kb.dot(m2), -kb.dot(m1)};
}

auto DiscreteElasticRod::ComputeGradW(const Vec3<Real> &kb,
                                      const Mat3<Real> &gradkb,
                                      const Vec3<Real> &m1,
                                      const Vec3<Real> &m2,
                                      const Vec3<Real> &psi,
                                      const Vec2<Real> &w) -> Mat2x3<Real> {
  // J is a 90 degree rotation
  Mat2<Real> J;
  J << 0, -1, 1, 0;

  Mat2x3<Real> M;
  M.row(0) = m2.transpose();
  M.row(1) = -m1.transpose();

  return M * gradkb - J * w * psi.transpose();
}

auto DiscreteElasticRod::ComputePEPTheta(int ii, const Vec3<Real> &m1j,
                                         const Vec3<Real> &m2j,
                                         const Mat2<Real> &JtimesB) -> Real {
  Real output = 0.0;

  if (ii > 0) {
    Vec2<Real> wi = ComputeW(kbs.at(ii), m1j, m2j);
    Real val = wi.transpose() * JtimesB * (wi - restNextCurvature.at(ii));
    val += 2 * BENDING_MODULUS * (thetas(ii) - thetas(ii - 1));
    val /= restLengths(ii);

    output += val;
  }

  if (ii < nRods - 1) {
    Vec2<Real> wi = ComputeW(kbs.at(ii), m1j, m2j);
    Real val = wi.transpose() * JtimesB * (wi - restPrevCurvature.at(ii));
    val -= 2 * BENDING_MODULUS * (thetas(ii + 1) - thetas(ii));
    val /= restLengths(ii + 1);

    output += val;
  }

  return output;
}
