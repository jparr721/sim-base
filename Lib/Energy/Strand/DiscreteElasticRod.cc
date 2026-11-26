#include <Energy/Strand/DiscreteElasticRod.h>

DiscreteElasticRod::DiscreteElasticRod(Mat<Real> vertices)
    : nRods(vertices.rows() - 1), vertices(std::move(vertices)) {
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

auto DiscreteElasticRod::ComputeCenterlineForcesStraight() -> Vec<Real> {
  Vec<Real> R = Vec<Real>::Zero(DOFs());

  const Real outer = -2 * gTwistingModulus / restEdges.at(0).length;
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const auto &kbGradient = kbGradients.at(ii);
    const auto &kb = kbs.at(ii);
    R.segment<3>(3 * ii) += outer * kbGradient.prev.transpose() * kb;
    R.segment<3>(3 * (ii + 1)) += outer * kbGradient.mid.transpose() * kb;
    R.segment<3>(3 * (ii + 2)) += outer * kbGradient.next.transpose() * kb;
  }

  const Real thetaDiff = thetas(thetas.rows() - 1) - thetas(0);
  const Real twistContrib = 2 * gTwistingModulus * thetaDiff / totalRestLength;
  for (int ii = 0; ii < nRods + 1; ++ii) {
    R.segment<3>(3 * ii) += twistContrib * holonomyGradients.at(ii);
  }

#ifndef NDEBUG
  std::cout << "forces: " << R.transpose() << std::endl;
#endif

  // Compute spring-mass forces
  for (int ii = 0; ii < nRods; ++ii) {
    Vec6<Real> x;
    x.segment<3>(0) = vertices.row(ii);
    x.segment<3>(3) = vertices.row(ii + 1);

    const MassSpring spring{.k = springStiffness,
                            .restLength = restEdges.at(ii).length};
    const Vec6<Real> force = -spring.Gradient(x);

    // Scatter the forces into the global vector
    R.segment<3>(3 * ii) += force.segment<3>(0);
    R.segment<3>(3 * (ii + 1)) += force.segment<3>(3);
  }

  return R;
}

void DiscreteElasticRod::UpdateBishopFrames() {
  // Time-parallel transport of the first frame onto the moved first edge.
  auto &first = frames.at(0);
  const Vec3<Real> &t = edges.at(0).t;
  const Vec3<Real> u =
      (RotationMatrixAroundNormal(2 * first.t.cross(t) / (1 + first.t.dot(t))) *
       first.u)
          .normalized();
  first = BishopFrame(t, u, t.cross(u));

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

void DiscreteElasticRod::UpdateKbGradients() {
  kbGradients.resize(nRods - 1);
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const Vec3<Real> &e0 = edges.at(ii).e;
    const Vec3<Real> &e1 = edges.at(ii + 1).e;
    const Vec3<Real> &kb = kbs.at(ii);
    const Real denominator =
        restEdges.at(ii).length * restEdges.at(ii + 1).length + e0.dot(e1);

    auto &gradient = kbGradients.at(ii);
    gradient.prev =
        (2 * CrossProductMatrix(e1) + kb * e1.transpose()) / denominator;
    gradient.next =
        (2 * CrossProductMatrix(e0) - kb * e0.transpose()) / denominator;
    gradient.mid = -(gradient.prev + gradient.next);
  }
}

void DiscreteElasticRod::UpdateHolonomyGradient() {
  holonomyGradients.assign(nRods + 1, Vec3<Real>::Zero());
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const Vec3<Real> prev = kbs.at(ii) / (2 * restEdges.at(ii).length);
    const Vec3<Real> next = -kbs.at(ii) / (2 * restEdges.at(ii + 1).length);
    holonomyGradients.at(ii) += prev;
    holonomyGradients.at(ii + 1) -= prev + next;
    holonomyGradients.at(ii + 2) += next;
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

void DiscreteElasticRod::UpdateLengths() { edges = ComputeEdges(vertices); }
