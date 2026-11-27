#include <Energy/Strand/DiscreteElasticRod.h>

namespace sim {

namespace {

auto CurvatureBinormal(const Vec3<Real> &e0, const Vec3<Real> &e1,
                       Real restLength0, Real restLength1) -> Vec3<Real> {
  return 2.0 * e0.cross(e1) / (restLength0 * restLength1 + e0.dot(e1));
}

auto CurvatureBinormalGradient(const Vec3<Real> &e0, const Vec3<Real> &e1,
                               const Vec3<Real> &kb, Real restLength0,
                               Real restLength1) -> KbGradient {
  const Real denominator = restLength0 * restLength1 + e0.dot(e1);
  KbGradient gradient;
  gradient.prev =
      (2 * CrossProductMatrix(e1) + kb * e1.transpose()) / denominator;
  gradient.next =
      (2 * CrossProductMatrix(e0) - kb * e0.transpose()) / denominator;
  gradient.mid = -(gradient.prev + gradient.next);
  return gradient;
}

// The stencil's two edges, its binormal and the binormal's gradient.
struct Curvature {
  Vec3<Real> e0;
  Vec3<Real> e1;
  Real denominator;
  Vec3<Real> kb;
  KbGradient gradient;
};

auto CurvatureOf(const VecN<9> &x, Real restLength0, Real restLength1)
    -> Curvature {
  Curvature c;
  c.e0 = x.segment<3>(3) - x.segment<3>(0);
  c.e1 = x.segment<3>(6) - x.segment<3>(3);
  c.denominator = restLength0 * restLength1 + c.e0.dot(c.e1);
  c.kb = CurvatureBinormal(c.e0, c.e1, restLength0, restLength1);
  c.gradient =
      CurvatureBinormalGradient(c.e0, c.e1, c.kb, restLength0, restLength1);
  return c;
}

// Stacks the three 3x3 vertex blocks of a 3 x 9 Jacobian.
auto Stacked(const KbGradient &g) -> Eigen::Matrix<Real, 3, 9> {
  Eigen::Matrix<Real, 3, 9> out;
  out << g.prev, g.mid, g.next;
  return out;
}

} // namespace

DiscreteElasticRod::DiscreteElasticRod(Mat<Real> vertices)
    : nRods(vertices.rows() - 1), vertices(std::move(vertices)) {
  SIM_ASSERT(this->vertices.rows() >= 3, "At least 3 nodes are required");
  restVertices = this->vertices;
  Initialize();
}

void DiscreteElasticRod::Initialize() {
  edges = ComputeEdges(vertices);
  restEdges = edges;
  totalRestLength = 0.0;
  for (const auto &edge : restEdges) {
    totalRestLength += edge.length;
  }

  // Thetas are per bend frame
  thetas = Vec<Real>::Zero(nRods - 1);

  BishopFrame initialFrame;
  initialFrame.t = vertices.row(1) - vertices.row(0);
  initialFrame.t.normalize();
  initialFrame.u = OrthogonalVectorUnsafe(initialFrame.t);
  initialFrame.v = initialFrame.t.cross(initialFrame.u);
  frames.resize(nRods);
  frames.at(0) = initialFrame;

  Advance(Positions(), 0);
}

void DiscreteElasticRod::Advance(const Vec<Real> &x, Real dt) {
  vertices = RowwiseUnFlatten(x, vertices.rows(), vertices.cols());
  thetas(0) += leftAngularVelocity * dt;
  thetas(thetas.rows() - 1) += rightAngularVelocity * dt;
  edges = ComputeEdges(vertices);
  kbs.resize(nRods - 1);
  for (int ii = 0; ii < nRods - 1; ++ii) {
    kbs.at(ii) = CurvatureBinormal(edges.at(ii).e, edges.at(ii + 1).e,
                                   restEdges.at(ii).length,
                                   restEdges.at(ii + 1).length);
  }
  UpdateBishopFrames();
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

auto DiscreteElasticRod::Stencil(const Vec<Real> &x, int vertex) -> VecN<9> {
  return x.segment<9>(3 * vertex);
}

auto DiscreteElasticRod::BendingAt(int vertex) const -> Bending {
  return {.modulus = gBendingModulus,
          .restLength0 = restEdges.at(vertex).length,
          .restLength1 = restEdges.at(vertex + 1).length};
}

auto DiscreteElasticRod::TwistAt(int vertex) const -> TwistForce {
  const Real thetaDiff = thetas(thetas.rows() - 1) - thetas(0);
  return {.scale = 2 * gTwistingModulus * thetaDiff / totalRestLength,
          .restLength0 = restEdges.at(vertex).length,
          .restLength1 = restEdges.at(vertex + 1).length};
}

auto DiscreteElasticRod::SpringAt(int edge) const -> MassSpring {
  return {.k = springStiffness, .restLength = restEdges.at(edge).length};
}

// Accumulated in the order the explicit strand integrator always used, so a
// run's floating-point results are unchanged.
auto DiscreteElasticRod::Forces(const Vec<Real> &x) const -> Vec<Real> {
  Vec<Real> R = Vec<Real>::Zero(DOFs());
  std::vector<Vec3<Real>> holonomyGradients(nRods + 1, Vec3<Real>::Zero());

  // Bending E = sum_i alpha |kb_i|^2 / lbar_i, with lbar_i the rest lengths of
  // the two edges meeting at kb_i (Bergou et al. 2008).
  for (int ii = 0; ii < nRods - 1; ++ii) {
    const Real outer =
        -2 * gBendingModulus /
        (restEdges.at(ii).length + restEdges.at(ii + 1).length);
    const Curvature c = CurvatureOf(Stencil(x, ii), restEdges.at(ii).length,
                                    restEdges.at(ii + 1).length);
    R.segment<3>(3 * ii) += outer * c.gradient.prev.transpose() * c.kb;
    R.segment<3>(3 * (ii + 1)) += outer * c.gradient.mid.transpose() * c.kb;
    R.segment<3>(3 * (ii + 2)) += outer * c.gradient.next.transpose() * c.kb;
    const Vec3<Real> prev = c.kb / (2 * restEdges.at(ii).length);
    const Vec3<Real> next = -c.kb / (2 * restEdges.at(ii + 1).length);
    holonomyGradients.at(ii) += prev;
    holonomyGradients.at(ii + 1) -= prev + next;
    holonomyGradients.at(ii + 2) += next;
  }

  const Real thetaDiff = thetas(thetas.rows() - 1) - thetas(0);
  const Real twistContrib = 2 * gTwistingModulus * thetaDiff / totalRestLength;
  for (int ii = 0; ii < nRods + 1; ++ii) {
    R.segment<3>(3 * ii) += twistContrib * holonomyGradients.at(ii);
  }

  for (int ii = 0; ii < nRods; ++ii) {
    const VecN<6> spring = -SpringAt(ii).Gradient(x.segment<6>(3 * ii));
    R.segment<3>(3 * ii) += spring.segment<3>(0);
    R.segment<3>(3 * (ii + 1)) += spring.segment<3>(3);
  }
  return R;
}

auto DiscreteElasticRod::Energy(const Vec<Real> &x) const -> Real {
  Real energy = 0;
  for (int ii = 0; ii < nRods - 1; ++ii) {
    energy += BendingAt(ii).Psi(Stencil(x, ii));
  }
  for (int ii = 0; ii < nRods; ++ii) {
    energy += SpringAt(ii).Psi(x.segment<6>(3 * ii));
  }
  const Real thetaDiff = thetas(thetas.rows() - 1) - thetas(0);
  return energy + gTwistingModulus * thetaDiff * thetaDiff / totalRestLength;
}

auto Bending::Psi(const VecN<9> &x) const -> Real {
  const Curvature c = CurvatureOf(x, restLength0, restLength1);
  return modulus * c.kb.squaredNorm() / (restLength0 + restLength1);
}

auto Bending::Gradient(const VecN<9> &x) const -> VecN<9> {
  const Curvature c = CurvatureOf(x, restLength0, restLength1);
  return 2 * modulus / (restLength0 + restLength1) *
         (Stacked(c.gradient).transpose() * c.kb);
}

// In the edge variables e = (e0, e1), with J_k = dkb/de_k:
// J_0 = -(2 [e1]x + kb e1^T) / D and J_1 = (2 [e0]x - kb e0^T) / D, with
// D = lbar0 lbar1 + e0 . e1. The Hessian of |kb|^2 is 2 (J^T J + G), where G
// differentiates A = J_0^T l and B = J_1^T l with l held at kb:
//   dA/de0 = -(e1 A^T + A e1^T) / D
//   dA/de1 = -(2 [l]x + e1 B^T + (kb . l) I + A e0^T) / D
//   dB/de0 =  (2 [l]x - e0 A^T - (kb . l) I - B e1^T) / D
//   dB/de1 = -(e0 B^T + B e0^T) / D
// and dA/de1 = (dB/de0)^T. e0 = x1 - x0 and e1 = x2 - x1 carry it to x.
auto Bending::Hessian(const VecN<9> &x) const -> MatN<9> {
  const Curvature c = CurvatureOf(x, restLength0, restLength1);
  const Real d = c.denominator;
  const Mat3<Real> j0 = -c.gradient.prev;
  const Mat3<Real> &j1 = c.gradient.next;
  const Vec3<Real> a = j0.transpose() * c.kb;
  const Vec3<Real> b = j1.transpose() * c.kb;
  const Real mu = c.kb.squaredNorm();
  const Mat3<Real> l = CrossProductMatrix(c.kb);
  const Mat3<Real> identity = Mat3<Real>::Identity();

  Eigen::Matrix<Real, 3, 6> j;
  j << j0, j1;
  MatN<6> g;
  g.topLeftCorner<3, 3>() =
      -(c.e1 * a.transpose() + a * c.e1.transpose()) / d;
  g.topRightCorner<3, 3>() = -(2 * l + c.e1 * b.transpose() + mu * identity +
                               a * c.e0.transpose()) / d;
  g.bottomLeftCorner<3, 3>() = g.topRightCorner<3, 3>().transpose();
  g.bottomRightCorner<3, 3>() =
      -(c.e0 * b.transpose() + b * c.e0.transpose()) / d;
  const MatN<6> hessianE =
      2 * modulus / (restLength0 + restLength1) * (j.transpose() * j + g);

  Eigen::Matrix<Real, 6, 9> edges = Eigen::Matrix<Real, 6, 9>::Zero();
  edges.block<3, 3>(0, 0) = -identity;
  edges.block<3, 3>(0, 3) = identity;
  edges.block<3, 3>(3, 3) = -identity;
  edges.block<3, 3>(3, 6) = identity;
  return edges.transpose() * hessianE * edges;
}

auto Bending::ClampedHessian(const VecN<9> &x) const -> MatN<9> {
  return ProjectToPsd<9>(Hessian(x));
}

namespace {

// The holonomy gradient's weights on the stencil's vertices.
auto HolonomyWeights(Real restLength0, Real restLength1)
    -> Eigen::Matrix<Real, 9, 3> {
  const Real prev = 1 / (2 * restLength0);
  const Real next = -1 / (2 * restLength1);
  Eigen::Matrix<Real, 9, 3> w;
  w << prev * Mat3<Real>::Identity(), -(prev + next) * Mat3<Real>::Identity(),
      next * Mat3<Real>::Identity();
  return w;
}

} // namespace

// The force is scale W kb, with W the holonomy weights, so its Jacobian is
// scale W dkb/dx.
auto TwistForce::Stiffness(const VecN<9> &x) const -> MatN<9> {
  const Curvature c = CurvatureOf(x, restLength0, restLength1);
  return -scale * HolonomyWeights(restLength0, restLength1) *
         Stacked(c.gradient);
}

auto TwistForce::ClampedStiffness(const VecN<9> &x) const -> MatN<9> {
  const MatN<9> k = Stiffness(x);
  return ProjectToPsd<9>((k + k.transpose()) / 2);
}

} // namespace sim
