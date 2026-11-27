// DerSystem against finite differences of its own forces and energy, the PSD
// clamp of its stiffness, and the Rayleigh form of the old strand damping.

#include <Solid/Integrators.h>
#include <Strand/DerSystem.h>

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

using namespace sim;

namespace {

const std::vector<Real> kSpacing = {0.7, 1.3, 0.9, 1.1, 0.8, 1.0};

auto RestRod() -> Mat<Real> {
  const int n = std::ssize(kSpacing) + 1;
  Mat<Real> x = Mat<Real>::Zero(n, 3);
  for (int i = 1; i < n; ++i) {
    x(i, 0) = x(i - 1, 0) + kSpacing[i - 1];
  }
  return x;
}

// Each edge turns from +x by up to `spread` (in units of a unit vector's
// components) and is stretched or compressed by up to `strain`.
auto Bent(unsigned seed, Real spread, Real strain) -> Vec<Real> {
  std::srand(seed);
  Mat<Real> x = RestRod();
  for (int i = 1; i < x.rows(); ++i) {
    const Vec3<Real> dir =
        (Vec3<Real>::UnitX() + spread * Vec3<Real>::Random()).normalized();
    const Real length = kSpacing[i - 1] * (1 + strain * Vec<Real>::Random(1)(0));
    x.row(i) = x.row(i - 1) + length * dir.transpose();
  }
  return RowwiseFlatten(x);
}

struct Case {
  std::string name;
  Vec<Real> x;
};

auto Cases() -> std::vector<Case> {
  return {{"near-straight", Bent(3, 1e-3, 0.01)},
          {"random", Bent(5, 0.6, 0.1)},
          {"highly bent", Bent(9, 3, 0.1)}};
}

// Every turning angle of a "highly bent" rod, so the case means what it says.
auto MaxTurningAngle(const Vec<Real> &x) -> Real {
  Real most = 0;
  for (int i = 0; i + 9 <= x.size(); i += 3) {
    const Vec3<Real> e0 = x.segment<3>(i + 3) - x.segment<3>(i);
    const Vec3<Real> e1 = x.segment<3>(i + 6) - x.segment<3>(i + 3);
    most = std::max(most, std::acos(e0.normalized().dot(e1.normalized())));
  }
  return most;
}

auto FdJacobian(const std::function<Vec<Real>(const Vec<Real> &)> &f,
                const Vec<Real> &x) -> Mat<Real> {
  constexpr Real h = 1e-6;
  Mat<Real> j(f(x).size(), x.size());
  for (int c = 0; c < x.size(); ++c) {
    Vec<Real> plus = x;
    Vec<Real> minus = x;
    plus(c) += h;
    minus(c) -= h;
    j.col(c) = (f(plus) - f(minus)) / (2 * h);
  }
  return j;
}

auto FdGradient(const std::function<Real(const Vec<Real> &)> &f,
                const Vec<Real> &x) -> Vec<Real> {
  return FdJacobian([&](const Vec<Real> &y) { return Vec<Real>::Constant(1, f(y)); },
                    x)
      .row(0)
      .transpose();
}

auto RelErr(const Mat<Real> &got, const Mat<Real> &want) -> Real {
  return (got - want).norm() / std::max(got.norm(), want.norm());
}

// A rod at rest shape RestRod(), with spring stiffness k and end twist
// theta^n - theta^0 = twist, moved to x.
auto Rod(Real k, Real twist) -> DiscreteElasticRod {
  DiscreteElasticRod rod(RestRod());
  rod.springStiffness = k;
  rod.thetas(rod.thetas.rows() - 1) = twist;
  return rod;
}

auto Dense(const SparseMat<Real> &m) -> Mat<Real> { return Mat<Real>(m); }

auto System(const DiscreteElasticRod &rod) -> DerSystem {
  return {rod, {}, NoDamping{}, 5, 0.3};
}

// Vertex 3 hangs 0.1 over a triangle, inside its face, and vertex 5 sits 0.2
// below another. Both triangles are wound counterclockwise seen from +y.
auto Contacts(const Vec<Real> &x) -> std::vector<SoftContact> {
  const auto under = [&](int vertex, Real dy) -> SoftContact {
    const Vec3<Real> p = x.segment<3>(3 * vertex) - Vec3<Real>(0, dy, 0);
    return {.vertex = vertex,
            .triangle = {p + Vec3<Real>(-0.5, 0, 0.4),
                         p + Vec3<Real>(0.5, 0, 0.3),
                         p + Vec3<Real>(0.1, 0, -0.6)}};
  };
  return {under(3, 0.1), under(5, -0.2)};
}

auto Forces(const DerSystem &system) {
  return [&](const Vec<Real> &y) -> Vec<Real> {
    return system.Forces(y, Vec<Real>::Zero(y.size()));
  };
}

TEST(DerSystem, HighlyBentCaseTurnsPastADoubleRightAngle) {
  EXPECT_GT(MaxTurningAngle(Cases()[2].x), 2.0);
  EXPECT_LT(MaxTurningAngle(Cases()[0].x), 0.01);
}

// Springs and twist off: the forces are bending alone.
TEST(DerSystem, BendingHessianIsMinusFdOfForces) {
  const auto rod = Rod(0, 0);
  const auto system = System(rod);
  for (const auto &[name, x] : Cases()) {
    EXPECT_LE(RelErr(Dense(system.Stiffness(x, false)),
                     -FdJacobian(Forces(system), x)),
              1e-6)
        << name;
  }
}

// The twist force alone, as the difference of two rods that differ only in
// their end angle.
TEST(DerSystem, TwistStiffnessIsMinusFdOfForces) {
  const auto twisted = Rod(0, 1.3);
  const auto untwisted = Rod(0, 0);
  const auto withTwist = System(twisted);
  const auto withoutTwist = System(untwisted);
  const auto twist = [&](const Vec<Real> &y) -> Vec<Real> {
    return Forces(withTwist)(y) - Forces(withoutTwist)(y);
  };
  for (const auto &[name, x] : Cases()) {
    const Mat<Real> k = Dense(withTwist.Stiffness(x, false)) -
                        Dense(withoutTwist.Stiffness(x, false));
    EXPECT_GT(k.norm(), 1e-3) << name;
    EXPECT_LE(RelErr(k, -FdJacobian(twist, x)), 1e-6) << name;
  }
}

TEST(DerSystem, StiffnessWithSpringsTwistAndContactsIsMinusFdOfForces) {
  const auto rod = Rod(100, 0.8);
  for (const auto &[name, x] : Cases()) {
    auto system = System(rod);
    system.SetContacts(Contacts(x));
    EXPECT_LE(RelErr(Dense(system.Stiffness(x, false)),
                     -FdJacobian(Forces(system), x)),
              1e-6)
        << name;
  }
}

// Collider triangles are wound counterclockwise seen from outside, so a
// vertex 0.1 above one, inside the 0.3 envelope, is pushed up and out.
TEST(DerSystem, ContactPushesOutOfACounterclockwiseTriangle) {
  const auto rod = Rod(100, 0);
  const Vec<Real> x = Cases()[1].x;
  auto system = System(rod);
  const Vec<Real> free = Forces(system)(x);
  system.SetContacts({Contacts(x)[0]});
  const Vec<Real> push = Forces(system)(x) - free;
  EXPECT_GT(push(3 * 3 + 1), 0);
  EXPECT_LE(push.norm(), std::abs(push(3 * 3 + 1)) * (1 + 1e-12));
}

// With the twist off, every force is conservative.
TEST(DerSystem, ForcesAreMinusFdOfEnergyWithoutTwist) {
  const auto rod = Rod(100, 0);
  for (const auto &[name, x] : Cases()) {
    auto system = System(rod);
    system.SetContacts(Contacts(x));
    EXPECT_LE(RelErr(Forces(system)(x),
                     -FdGradient([&](const Vec<Real> &y) {
                       return system.Energy(y);
                     }, x)),
              1e-6)
        << name;
  }
}

// The positive part of a symmetric h, from its SVD rather than its
// eigendecomposition: |h| = V S V^T, and the positive part is (h + |h|) / 2.
auto PositivePart(const Mat<Real> &h) -> Mat<Real> {
  const Mat<Real> sym = (h + h.transpose()) / 2;
  const Eigen::JacobiSVD<Mat<Real>> svd(sym, Eigen::ComputeFullV);
  const Mat<Real> abs =
      svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  return (sym + abs) / 2;
}

auto MinEigenvalue(const Mat<Real> &h) -> Real {
  return Eigen::SelfAdjointEigenSolver<Mat<Real>>((h + h.transpose()) / 2)
      .eigenvalues()
      .minCoeff();
}

// Each bending and twist block is projected on its own: clamped equals the
// positive part of the unclamped block.
TEST(DerSystem, ClampedBlocksArePositiveParts) {
  const auto rod = Rod(100, 0.8);
  for (const auto &[name, x] : Cases()) {
    for (int i = 0; i < rod.nRods - 1; ++i) {
      const VecN<9> s = DiscreteElasticRod::Stencil(x, i);
      const Bending bending = rod.BendingAt(i);
      const TwistForce twist = rod.TwistAt(i);
      EXPECT_LE(RelErr(bending.ClampedHessian(s),
                       PositivePart(bending.Hessian(s))),
                1e-10)
          << name << " bend " << i;
      EXPECT_LE(RelErr(twist.ClampedStiffness(s),
                       PositivePart(twist.Stiffness(s))),
                1e-10)
          << name << " twist " << i;
    }
  }
}

// The clamped stiffness is PSD, and the unclamped one is not, as the control.
TEST(DerSystem, ClampedStiffnessIsPsdAndUnclampedIsNot) {
  const auto rod = Rod(100, 0.8);
  for (const auto &[name, x] : Cases()) {
    auto system = System(rod);
    system.SetContacts(Contacts(x));
    const Mat<Real> clamped = Dense(system.Stiffness(x, true));
    EXPECT_LE((clamped - clamped.transpose()).norm(), 1e-12 * clamped.norm())
        << name;
    EXPECT_GE(MinEigenvalue(clamped), -1e-9 * clamped.norm()) << name;
    EXPECT_LT(MinEigenvalue(Dense(system.Stiffness(x, false))),
              -1e-6 * clamped.norm())
        << name;
  }
}

// Each kind of block on its own has a negative eigenvalue somewhere, so each
// clamp has something to do.
TEST(DerSystem, EachUnclampedKindIsIndefiniteSomewhere) {
  const auto rod = Rod(100, 0.8);
  Real bend = 0;
  Real twist = 0;
  for (const auto &[name, x] : Cases()) {
    for (int i = 0; i < rod.nRods - 1; ++i) {
      const VecN<9> s = DiscreteElasticRod::Stencil(x, i);
      bend = std::min(bend, MinEigenvalue(rod.BendingAt(i).Hessian(s)));
      twist = std::min(twist, MinEigenvalue(rod.TwistAt(i).Stiffness(s)));
    }
  }
  EXPECT_LT(bend, -1e-6);
  EXPECT_LT(twist, -1e-6);
}

// Rayleigh damping: D = mass M + stiffness K(x) with K clamped and without
// contacts, and Forces(x, v) = Forces(x, 0) - D v.
TEST(DerSystem, RayleighDampingIsLinearInVelocity) {
  const auto rod = Rod(100, 0.8);
  const Vec<Real> x = Cases()[1].x;
  DerSystem system(rod, {0}, Rayleigh{.mass = 3, .stiffness = 0.01}, 5, 0.3);
  system.SetContacts(Contacts(x));
  auto undamped = System(rod);
  const SparseMat<Real> want =
      3 * system.Mass() + 0.01 * undamped.Stiffness(x, true);
  EXPECT_LE(RelErr(Dense(system.DampingMatrix(x)), Dense(want)), 1e-14);
  std::srand(2);
  const Vec<Real> v = Vec<Real>::Random(x.size());
  EXPECT_LE(RelErr(system.Forces(x, v),
                   system.Forces(x, Vec<Real>::Zero(x.size())) -
                       system.DampingMatrix(x) * v),
            1e-14);
  EXPECT_EQ(system.Constraints().pinned, (std::vector<int>{0}));
  EXPECT_EQ(Dense(System(rod).DampingMatrix(x)).norm(), 0);
}

// The vertex mass inverts exactly, so an explicit step's M^-1 is the 5 the
// old strand integrator stored.
TEST(DerSystem, MassIsTheOldInverseMassInverted) {
  const auto rod = Rod(100, 0);
  const auto system = System(rod);
  EXPECT_EQ(Vec<Real>(system.Mass().diagonal()).cwiseInverse(),
            Vec<Real>::Constant(system.Dofs(), 5));
}

// The old explicit strand step, x += dt^2 M^-1 f + dt w followed by the
// velocity filter w = (I - dt c M^-1) dx / dt, is ForwardEuler under
// Rayleigh mass damping c / m. Substituting w_n:
//   dx_{n+1} = dt^2 M^-1 f + (I - dt c M^-1) dx_n
//            = dt^2 M^-1 (f - (c / m) M v_n) + dt v_n,  v_n = dx_n / dt,
// which is ForwardEuler with Forces = f - D v and D = (c / m) M.
TEST(DerSystem, VelocityFilterIsRayleighMassDamping) {
  constexpr Real c = 0.75;
  constexpr Real dt = 1e-3;
  const auto rod = Rod(1000, 0.4);
  const DerSystem undamped(rod, {0}, NoDamping{}, 5, 0.3);
  const DerSystem damped(rod, {0}, Rayleigh{.mass = c / rod.vertexMass}, 5,
                         0.3);
  const Vec<Real> start = Bent(5, 0.3, 0.01);
  Vec<Real> external = Vec<Real>::Zero(start.size());
  for (int i = 0; i < external.size(); i += 3) {
    external(i + 1) = -0.981;
  }

  Vec<Real> x = start;
  Vec<Real> w = Vec<Real>::Zero(x.size());
  IntegratorState state{.x = start, .v = Vec<Real>::Zero(start.size())};
  for (int step = 0; step < 2000; ++step) {
    Vec<Real> dx = dt * dt / rod.vertexMass *
                       (undamped.Forces(x, Vec<Real>::Zero(x.size())) +
                        external) +
                   dt * w;
    dx.head<3>().setZero();
    w = (1 - dt * c / rod.vertexMass) * dx / dt;
    x += dx;
    Step(ForwardEuler{}, damped, state, external, dt);
  }
  EXPECT_GT((x - start).norm(), 0.1);
  EXPECT_LE((state.x - x).cwiseAbs().maxCoeff(), 1e-12);
}

} // namespace
