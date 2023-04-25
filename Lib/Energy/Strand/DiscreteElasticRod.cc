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

  // Init the bends
  bends.resize(nRods - 1);
  restBends.resize(nRods - 1);

  // Get the curvature update
  UpdateMaterialCurvatures();

  // Set rest curvatures
  restBends = bends;

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

  // Formula in 7.1, the General case - dEdx
  for (int ii = 0; ii < nRods - 1; ++ii) {
    Vec3<Real> vertexForce = Vec3<Real>::Zero();

    for (int jj = ii; jj <= ii + 1; ++jj) {
      const auto &w =
          jj == ii ? bends.at(ii).prevCurvature : bends.at(ii).nextCurvature;
      const auto &wbar = jj == ii ? restBends.at(ii).prevCurvature
                                  : bends.at(ii).nextCurvature;

      const Mat2x3<Real> gradW = ComputeGradW(gradientkbs.at(ii), m1s.at(jj),
                                              m2s.at(jj), holonomy.at(ii), w);

      vertexForce += gradW.transpose() * Bhat * (w - wbar);
    }

    vertexForce /= restLengths(ii);

    R.segment<3>(3 * ii) += vertexForce;
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
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const Vec3<Real> kb = kbs.at(ii);
    const Vec3<Real> m1 = m1s.at(ii);
    const Vec3<Real> m2 = m2s.at(ii);

    bends.at(ii).prevCurvature = ComputeW(kb, m1, m2);

    if (ii > 0) {
      bends.at(ii - 1).nextCurvature = ComputeW(kb, m1, m2);
    }
  }
}

void DiscreteElasticRod::UpdateQuasistaticMaterialFrame() {
  // J is a 90 degree rotation matrix
  Mat2<Real> J;
  J << 0, -1, 1, 0;
  const Mat2<Real> Bhat = Mat2<Real>::Identity() * BENDING_MODULUS;

  Real dEdTheta = 0.0;
  Vec<Real> lowerDiagonal = Vec<Real>::Zero(nRods);
  Vec<Real> upperDiagonal = Vec<Real>::Zero(nRods);
  Vec<Real> diagonal = Vec<Real>::Zero(nRods);

  const auto computeGradWi = [&J, &Bhat](const Vec2<Real> &w,
                                         const Vec2<Real> &wbar,
                                         Real restLength) -> Real {
    const Real invRestLength = 1.0 / restLength;
    return invRestLength * w.transpose() * J * Bhat * (w - wbar);
  };

  // 8 newton steps
  for (int iter = 0; iter < 8; ++iter) {
    for (int ii = 0; ii < nRods - 1; ++ii) {
      // Previous curvature W_j + 2beta * (mj/lj)
      {
        const Vec2<Real> w = bends.at(ii).prevCurvature;
        const Vec2<Real> wbar = restBends.at(ii).prevCurvature;
        const Real restLength = restLengths(ii);
        const Real mj = thetas(ii + 1) - thetas(ii);
        const Real Wi = computeGradWi(w, wbar, restLength);
        dEdTheta += Wi + 2 * BENDING_MODULUS * mj / restLength;
      }

      // Next curvature W_j+1 = 2 * beta * (mj+1/lj+1)
      if (ii > 0) {
        const Vec2<Real> w = bends.at(ii - 1).nextCurvature;
        const Vec2<Real> wbar = restBends.at(ii - 1).nextCurvature;
        const Real restLength = restLengths(ii - 1);
        const Real mj = thetas(ii) - thetas(ii - 1);
        const Real Wi = computeGradWi(w, wbar, restLength);
        dEdTheta += Wi + 2 * BENDING_MODULUS * mj / restLength;
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
  holonomy.resize(nRods - 1);
  holonomyLhs.resize(nRods - 1);
  holonomyRhs.resize(nRods - 1);

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

auto DiscreteElasticRod::ComputeGradW(const Mat3<Real> &gradkb,
                                      const Vec3<Real> &m1,
                                      const Vec3<Real> &m2,
                                      const Vec3<Real> &gradpsi,
                                      const Vec2<Real> &w) -> Mat2x3<Real> {
  // J is a 90 degree rotation
  Mat2<Real> J;
  J << 0, -1, 1, 0;

  Mat2x3<Real> M;
  M.row(0) = m2.transpose();
  M.row(1) = -m1.transpose();

  return M * gradkb - J * w * gradpsi.transpose();
}
