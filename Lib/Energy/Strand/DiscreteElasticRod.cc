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

  // Thetas are per bend frame
  thetas = Vec<Real>::Zero(nRods - 1);

  // Lengths are per edge
  lengths = Vec<Real>::Zero(nRods);
  restLengths = Vec<Real>::Zero(nRods);

  // Material frames are per bend frame
  m1s.resize(nRods - 1);
  m2s.resize(nRods - 1);

  UpdateLengths();
  restLengths = lengths;

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

  // Get the curvature update - don't bother with the quasistatic update here
  // since it's just 0 anyway.
  UpdateMaterialCurvatures();
  restCurvature = curvature;

  // Update the material curvature gradients
  UpdateKbGradients();

  // Update material holonomy
  UpdateHolonomyGradient();
}

auto DiscreteElasticRod::ComputeCenterlineForcesGeneral() -> Vec<Real> {
  Vec<Real> R = Vec<Real>::Zero(DOFs());

  // J is a 90 degree rotation matrix. Always
  Mat2<Real> J;
  J << 0, -1, 1, 0;

  // Compute the derivative of the curvature
  const Mat2<Real> Bhat = Mat2<Real>::Identity() * gBendingModulus;

  // Formula in 7.1, the General case - dEdx
  for (int ii = 0; ii < nRods - 1; ++ii) {
    Vec3<Real> vertexForce = Vec3<Real>::Zero();

    for (int jj = ii; jj <= ii + 1; ++jj) {
      const auto &w = curvature.at(ii);
      const auto &wbar = restCurvature.at(ii);

      const Mat2x3<Real> gradW = ComputeGradOmega(
          gradientkbs.at(ii), m1s.at(jj), m2s.at(jj), holonomy.at(ii), w);

      vertexForce += gradW.transpose() * Bhat * (w - wbar);
    }

    vertexForce /= restLengths(ii);

    R.segment<3>(3 * ii) += vertexForce;
  }

  return R;
}

auto DiscreteElasticRod::ComputeCenterlineForcesStraight() -> Vec<Real> {
  Vec<Real> R = Vec<Real>::Zero(DOFs());

  for (int ii = 0; ii < nRods - 2; ++ii) {
    const Vec3<Real> vertexForce = ComputepEpxi(ii);
    R.segment<3>(3 * ii + 1) += vertexForce;
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
  for (int ii = 0; ii < nRods - 1; ++ii) {
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
  curvature.clear();
  curvature.resize(nRods - 1);
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const Vec3<Real> kb = kbs.at(ii);
    const Vec3<Real> m1 = m1s.at(ii);
    const Vec3<Real> m2 = m2s.at(ii);

    curvature.at(ii) = ComputeOmega(kb, m1, m2);
  }
}

void DiscreteElasticRod::UpdateQuasistaticMaterialFrame() {
  // J is a 90 degree rotation matrix
  Mat2<Real> J;
  J << 0, -1, 1, 0;
  const Mat2<Real> Bhat = Mat2<Real>::Identity() * gBendingModulus;

  Vec<Real> dEdTheta = Vec<Real>::Zero(nRods - 1);
  Vec<Real> lowerDiagonal = Vec<Real>::Zero(nRods - 1);
  Vec<Real> upperDiagonal = Vec<Real>::Zero(nRods - 1);
  Vec<Real> diagonal = Vec<Real>::Zero(nRods - 1);

  // Run a single newton step
  const Vec<Real> twistForce = ComputeTwistingForce();
  ComputeTwistingHessian(upperDiagonal, diagonal, lowerDiagonal);

  // Compute the theta update
  thetas =
      FactorTriDiagonalMatrix(upperDiagonal, diagonal, lowerDiagonal, dEdTheta);

#ifndef NDEBUG
  std::cout << "new thetas: " << thetas.transpose() << std::endl;
  std::cout << "thetas.norm(): " << thetas.norm() << std::endl;
#endif

  // Compute the quasistatic frames with the new theta values
  UpdateMaterialFrames();
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
    gradientkbs.at(ii) = -(gradientLhskbs.at(ii) + gradientRhskbs.at(ii));
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
  kbs.resize(nRods - 1);
  for (int ii = 0; ii < nRods - 1; ++ii) {
    // Get the edges on either side of the vertex
    const Vec3<Real> e0 = vertices.row(ii + 1) - vertices.row(ii);
    const Vec3<Real> e1 = vertices.row(ii + 2) - vertices.row(ii + 1);

    // Discrete curvature binormal
    kbs.at(ii) = 2.0 * e0.cross(e1) /
                 (restLengths(ii) * restLengths(ii + 1) + e0.dot(e1));
  }
}

auto DiscreteElasticRod::ComputeOmega(const Vec3<Real> &kb,
                                      const Vec3<Real> &m1,
                                      const Vec3<Real> &m2) -> Vec2<Real> {
  return {kb.dot(m2), -kb.dot(m1)};
}

auto DiscreteElasticRod::ComputeGradOmega(const Mat3<Real> &gradkb,
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

auto DiscreteElasticRod::ComputeTwistingForce() -> Vec<Real> {
  Mat2<Real> J;
  J << 0, -1, 1, 0;
  const Mat2<Real> Bhat = Mat2<Real>::Identity() * gBendingModulus;

  Vec<Real> forces = Vec<Real>::Zero(nRods - 1);

  const auto gradW = [&J, &Bhat, this](int j) -> Real {
    const auto &w = curvature.at(j);
    const auto &wbar = restCurvature.at(j);
    const Real restLength = restLengths(j);
    const Real invRestLength = 1.0 / restLength;
    return invRestLength * w.transpose() * J * Bhat * (w - wbar);
  };

  // Directly from equation 7.
  for (int j = 0; j < nRods - 2; ++j) {
    const auto &Wj = gradW(j);
    const auto &Wj1 = gradW(j + 1);
    const Real thetaDiff = thetas(j) - thetas(j + 1);

    forces(j) = (Wj + Wj1) * 2 * gBendingModulus * thetaDiff / restLengths(j);
  }

#ifndef NDEBUG
  std::cout << "forces: " << forces.transpose() << std::endl;
#endif

  return forces;
}

void DiscreteElasticRod::ComputeTwistingHessian(Vec<Real> &upper,
                                                Vec<Real> &center,
                                                Vec<Real> &lower) {
  Mat<Real> hessian;

  Mat2<Real> J;
  J << 0, -1, 1, 0;
  const Mat2<Real> Bhat = Mat2<Real>::Identity() * gBendingModulus;
  const auto hessW = [&J, &Bhat, this](int j) -> Real {
    const auto &w = curvature.at(j);
    const auto &wbar = restCurvature.at(j);
    const Real restLength = restLengths(j);
    const Real invRestLength = 1.0 / restLength;

    const Real lhs = invRestLength * w.transpose() * J.transpose() * Bhat * w;
    const Real rhs = invRestLength * w.transpose() * Bhat * (w - wbar);
    return lhs - rhs;
  };

  // Components of the hessian tri-diagonal matrix
  lower = Vec<Real>::Zero(nRods - 1);
  upper = Vec<Real>::Zero(nRods - 1);
  center = Vec<Real>::Zero(nRods - 1);

  for (int j = 0; j < nRods - 2; ++j) {
    lower(j) = -2 * gBendingModulus / restLengths(j);
    upper(j) = -2 * gBendingModulus / restLengths(j + 1);

    center(j) =
        hessW(j) + hessW(j + 1) +
        2 * gBendingModulus * (1 / restLengths(j) + 1 / restLengths(j + 1));
  }
}

auto DiscreteElasticRod::ComputepEpxi(int j) -> Vec3<Real> {
  const auto scaledTwistContrib = 2 * gTwistingModulus / restLengths(j);
  const auto &gradKb = gradientkbs.at(j);
  const auto &kb = kbs.at(j);
  const Real totalRestLength = restLengths.sum();
  const auto &gradHolo = holonomy.at(j);
  const Real thetaDiff = thetas(thetas.rows() - 1) - thetas(0);
  return scaledTwistContrib * gradKb.transpose() * kb +
         (gBendingModulus * thetaDiff) / totalRestLength * gradHolo;
}

void DiscreteElasticRod::UpdateLengths() {
  for (int ii = 0; ii < nRods; ++ii) {
    const Real length = (vertices.row(ii + 1) - vertices.row(ii)).norm();
    lengths(ii) = length;
  }
}
