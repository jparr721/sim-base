#include <Strand/DerSystem.h>

#include <utility>

namespace sim {

namespace {

// The vertex, then the triangle.
auto ContactDofs(const Vec<Real> &x, const SoftContact &c) -> VecN<12> {
  VecN<12> stacked;
  stacked << x.segment<3>(3 * c.vertex), c.triangle[0], c.triangle[1],
      c.triangle[2];
  return stacked;
}

auto Area(const SoftContact &c) -> Real {
  return TriangleArea(c.triangle[0], c.triangle[1], c.triangle[2]);
}

template <int N>
void AddBlock(std::vector<Eigen::Triplet<Real>> &triplets, int firstDof,
              const MatN<N> &block) {
  for (int row = 0; row < N; ++row) {
    for (int col = 0; col < N; ++col) {
      triplets.emplace_back(firstDof + row, firstDof + col, block(row, col));
    }
  }
}

} // namespace

DerSystem::DerSystem(const DiscreteElasticRod &rod,
                     const std::vector<int> &pinned, Damping damping,
                     Real collisionStiffness, Real collisionEpsilon)
    : rod(rod), damping(std::move(damping)),
      collision(collisionStiffness, collisionEpsilon),
      mass(ConstructDiagonalSparseMatrix<Real>(
          Vec<Real>::Constant(rod.DOFs(), rod.vertexMass))),
      constraints{.dofs = rod.DOFs(), .pinned = pinned} {}

auto DerSystem::Energy(const Vec<Real> &x) const -> Real {
  Real energy = rod.Energy(x);
  for (const SoftContact &c : contacts) {
    energy += Area(c) * collision.Psi(ContactDofs(x, c));
  }
  return energy;
}

auto DerSystem::Forces(const Vec<Real> &x, const Vec<Real> &v) const
    -> Vec<Real> {
  Vec<Real> forces = rod.Forces(x);
  for (const SoftContact &c : contacts) {
    const VecN<12> force = -Area(c) * collision.Gradient(ContactDofs(x, c));
    forces.segment<3>(3 * c.vertex) += force.segment<3>(0);
  }
  if (std::holds_alternative<Rayleigh>(damping)) {
    forces -= DampingMatrix(x) * v;
  }
  return forces;
}

auto DerSystem::ElasticStiffness(const Vec<Real> &x, bool clamped) const
    -> std::vector<Eigen::Triplet<Real>> {
  std::vector<Eigen::Triplet<Real>> triplets;
  triplets.reserve(2 * 81 * (rod.nRods - 1) + 36 * rod.nRods);
  for (int ii = 0; ii < rod.nRods - 1; ++ii) {
    const VecN<9> stencil = DiscreteElasticRod::Stencil(x, ii);
    const Bending bending = rod.BendingAt(ii);
    const TwistForce twist = rod.TwistAt(ii);
    AddBlock<9>(triplets, 3 * ii,
                clamped ? bending.ClampedHessian(stencil)
                        : bending.Hessian(stencil));
    AddBlock<9>(triplets, 3 * ii,
                clamped ? twist.ClampedStiffness(stencil)
                        : twist.Stiffness(stencil));
  }
  for (int ii = 0; ii < rod.nRods; ++ii) {
    const VecN<6> edge = x.segment<6>(3 * ii);
    const MassSpring spring = rod.SpringAt(ii);
    AddBlock<6>(triplets, 3 * ii,
                clamped ? spring.ClampedHessian(edge) : spring.Hessian(edge));
  }
  return triplets;
}

// The triangles are static, so a contact adds only its vertex's 3x3 block.
auto DerSystem::Stiffness(const Vec<Real> &x, bool clamped) const
    -> SparseMat<Real> {
  std::vector<Eigen::Triplet<Real>> triplets = ElasticStiffness(x, clamped);
  for (const SoftContact &c : contacts) {
    const Mat3<Real> block =
        Area(c) * collision.Hessian(ContactDofs(x, c)).topLeftCorner<3, 3>();
    AddBlock<3>(triplets, 3 * c.vertex,
                clamped ? ProjectToPsd<3>(block) : block);
  }
  SparseMat<Real> k(Dofs(), Dofs());
  k.setFromTriplets(triplets.begin(), triplets.end());
  return k;
}

// mass M + stiffness K(x) for Rayleigh, with K the clamped elastic stiffness.
auto DerSystem::DampingMatrix(const Vec<Real> &x) const -> SparseMat<Real> {
  const auto *rayleigh = std::get_if<Rayleigh>(&damping);
  if (rayleigh == nullptr) {
    return SparseMat<Real>(Dofs(), Dofs());
  }
  SparseMat<Real> d = rayleigh->mass * mass;
  if (rayleigh->stiffness != 0) {
    const std::vector<Eigen::Triplet<Real>> triplets =
        ElasticStiffness(x, true);
    SparseMat<Real> k(Dofs(), Dofs());
    k.setFromTriplets(triplets.begin(), triplets.end());
    d += rayleigh->stiffness * k;
  }
  return d;
}

} // namespace sim
