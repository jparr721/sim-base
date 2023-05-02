#include <Energy/Strand/DiscreteElasticRod.h>

DiscreteElasticRod::DiscreteElasticRod(Mat<Real> vertices)
    : vertices(std::move(vertices)), nRods(vertices.rows() - 1),
      massSpring(new MassSpring(100000, 0.25)) {
  // At least 3 nodes is required
  ASSERT(this->vertices.rows() >= 3, "At least 3 nodes are required");

  // initial position of centerline in rest state
  restVertices = this->vertices;

  // Initialize the mass matrix - uniform mass right now.
  Vec<Real> masses = Vec<Real>::Ones(DOFs()) * 5;
  mInv = ConstructDiagonalSparseMatrix(masses);

  Initialize();
}

void DiscreteElasticRod::Initialize() {
  // Compute edges
  //  edges = ComputeEdges(vertices);
  UpdateLengths();
  restEdges = edges;
  totalRestLength = 0.0;
  for (const auto &edge : restEdges) {
    totalRestLength += edge.length;
  }

  // Thetas are per bend frame
  thetas = Vec<Real>::Zero(nRods - 1);

  // Material frames are per bend frame
  m1s.resize(nRods - 1);
  m2s.resize(nRods - 1);

  // Make initial frames
  BishopFrame initialFrame;
  // Get the normalized tangent vector
  initialFrame.t = vertices.row(1) - vertices.row(0);
  initialFrame.t.normalize();

  // Get u0 as a random orthogonal vector
  initialFrame.u = OrthogonalVectorUnsafe(initialFrame.t);

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

      const Mat2x3<Real> gradW =
          ComputeGradOmega(kbGradients.at(ii), m1s.at(jj), m2s.at(jj),
                           holonomyGradients.at(ii), w);

      vertexForce += gradW.transpose() * Bhat * (w - wbar);
    }

    vertexForce /= restEdges.at(ii).length;
    R.segment<3>(3 * ii) += vertexForce;
  }

  return R;
}

auto DiscreteElasticRod::ComputeCenterlineForcesStraight() -> Vec<Real> {
  Vec<Real> R = Vec<Real>::Zero(DOFs());

  //  for (int ii = 0; ii < nRods - 2; ++ii) {
  //    const Vec3<Real> vertexForce = ComputepEpxi(ii);
  //    R.segment<3>(3 * ii + 1) += vertexForce;
  //  }

  // Compute spring-mass forces
  for (int ii = 0; ii < nRods; ++ii) {
    Vec6<Real> x;
    x.segment<3>(0) = vertices.row(ii);
    x.segment<3>(3) = vertices.row(ii + 1);

    const Vec6<Real> force = -edges.at(ii).length * massSpring->Gradient(x);

    // Scatter the forces into the global vector
    R.segment<3>(3 * ii) += force.segment<3>(0);
    R.segment<3>(3 * (ii + 1)) += force.segment<3>(3);
  }

  return R;
}

void DiscreteElasticRod::UpdateBishopFrames() {
  ParallelTransportModifiedBishopFrames(kbs, edges, frames);
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
  kbGradients.resize(nRods);

  // Here [e] is a skew-symmetric 3 × 3 matrix acting on 3-vectors x by [e] · x
  // = e × x.
  for (int ii = 0; ii < nRods - 1; ++ii) {
    // Lhs
    //    const Vec3<Real> e0 = vertices.row(ii + 1) - vertices.row(ii);
    const Vec3<Real> e0 = edges.at(ii).e;
    const Mat3<Real> crossLhs = CrossProductMatrix(e0);

    // Rhs
    //    const Vec3<Real> e1 = vertices.row(ii + 2) - vertices.row(ii + 1);
    const Vec3<Real> e1 = edges.at(ii).e;
    const Mat3<Real> crossRhs = CrossProductMatrix(e1);

    // Compute the lhs gradient
    const Real denominator =
        restEdges.at(ii).length * restEdges.at(ii + 1).length + e0.dot(e1);

    // DEBUG: This could be a broken thing. The paper just _had_ to switch the
    // damn terms.
    const auto lhs = 2 * crossRhs + kbs.at(ii) * e1.transpose() / denominator;
    const auto rhs = 2 * crossLhs + kbs.at(ii) * e0.transpose() / denominator;
    kbGradients.at(ii) = -(lhs + rhs);
  }
}

void DiscreteElasticRod::UpdateHolonomyGradient() {
  holonomyGradients.resize(nRods - 1);
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const auto lhs = kbs.at(ii) / 2 * (restEdges.at(ii).length);
    const auto rhs = -kbs.at(ii) / 2 * (restEdges.at(ii + 1).length);
    holonomyGradients.at(ii) = -1 * (lhs + rhs);
  }
}

void DiscreteElasticRod::Computekbs() {
  kbs.resize(nRods - 1);
  for (int ii = 0; ii < nRods - 1; ++ii) {
    // Get the edges on either side of the vertex
    const Vec3<Real> e0 = vertices.row(ii + 1) - vertices.row(ii);
    const Vec3<Real> e1 = vertices.row(ii + 2) - vertices.row(ii + 1);

    // Discrete curvature binormal
    kbs.at(ii) =
        2.0 * e0.cross(e1) /
        (restEdges.at(ii).length * restEdges.at(ii + 1).length + e0.dot(e1));
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
    const Real restLength = restEdges.at(j).length;
    const Real invRestLength = 1.0 / restLength;
    return invRestLength * w.transpose() * J * Bhat * (w - wbar);
  };

  // Directly from equation 7.
  for (int j = 0; j < nRods - 2; ++j) {
    const auto &Wj = gradW(j);
    const auto &Wj1 = gradW(j + 1);
    const Real thetaDiff = thetas(j) - thetas(j + 1);

    forces(j) =
        (Wj + Wj1) * 2 * gBendingModulus * thetaDiff / restEdges.at(j).length;
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
    const Real restLength = restEdges.at(j).length;
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
    lower(j) = -2 * gBendingModulus / restEdges.at(j).length;
    upper(j) = -2 * gBendingModulus / restEdges.at(j + 1).length;

    center(j) =
        hessW(j) + hessW(j + 1) +
        2 * gBendingModulus *
            (1 / restEdges.at(j).length + 1 / restEdges.at(j + 1).length);
  }
}

auto DiscreteElasticRod::ComputepEpxi(int j) -> Vec3<Real> {
  const auto scaledTwistContrib = 2 * gTwistingModulus / restEdges.at(j).length;
  const auto &gradKb = kbGradients.at(j);
  const auto &kb = kbs.at(j);
  const auto &gradHolo = holonomyGradients.at(j);
  const Real thetaDiff = thetas(thetas.rows() - 1) - thetas(0);
  return scaledTwistContrib * gradKb.transpose() * kb +
         (gBendingModulus * thetaDiff) / totalRestLength * gradHolo;
}

void DiscreteElasticRod::UpdateLengths() { edges = ComputeEdges(vertices); }
