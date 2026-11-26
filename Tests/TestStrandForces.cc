// Finite-difference checks of the straight-rod strand forces against the
// energies they derive from (Bergou et al. 2008, Discrete Elastic Rods, plus
// the mass-spring inextensibility constraint). The energies here are written
// independently of the library: the forces must equal -grad E.

#include <Energy/Strand/DiscreteElasticRod.h>

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <vector>

namespace {

const std::vector<Real> kSpacing = {0.7, 1.3, 0.9, 1.1, 0.8};

auto Edge(const Mat<Real> &x, int i) -> Vec3<Real> {
  return x.row(i + 1) - x.row(i);
}

auto Kb(const Mat<Real> &x, int i) -> Vec3<Real> {
  const Vec3<Real> e0 = Edge(x, i);
  const Vec3<Real> e1 = Edge(x, i + 1);
  return 2 * e0.cross(e1) / (kSpacing[i] * kSpacing[i + 1] + e0.dot(e1));
}

auto RestRod() -> Mat<Real> {
  const int n = std::ssize(kSpacing) + 1;
  Mat<Real> x = Mat<Real>::Zero(n, 3);
  for (int i = 1; i < n; ++i) {
    x(i, 0) = x(i - 1, 0) + kSpacing[i - 1];
  }
  return x;
}

// Every edge keeps its rest length and turns to a random direction near +x.
auto InextensibleRod() -> Mat<Real> {
  std::srand(7);
  Mat<Real> x = RestRod();
  for (int i = 1; i < x.rows(); ++i) {
    const Vec3<Real> dir =
        (Vec3<Real>::UnitX() + 0.8 * Vec3<Real>::Random()).normalized();
    x.row(i) = x.row(i - 1) + kSpacing[i - 1] * dir.transpose();
  }
  return x;
}

auto StretchedRod() -> Mat<Real> {
  const Mat<Real> x = InextensibleRod();
  std::srand(11);
  return x + 0.1 * Mat<Real>::Random(x.rows(), 3);
}

auto FdGradient(const std::function<Real(const Mat<Real> &)> &energy,
                const Mat<Real> &x) -> Vec<Real> {
  constexpr Real h = 1e-6;
  Vec<Real> g(x.size());
  for (int r = 0; r < x.rows(); ++r) {
    for (int c = 0; c < 3; ++c) {
      Mat<Real> plus = x;
      Mat<Real> minus = x;
      plus(r, c) += h;
      minus(r, c) -= h;
      g(3 * r + c) = (energy(plus) - energy(minus)) / (2 * h);
    }
  }
  return g;
}

auto RelErr(const Vec<Real> &got, const Vec<Real> &want) -> Real {
  return (got - want).norm() / std::max(got.norm(), want.norm());
}

// Rotates u by the turning angle about a x b, taking unit tangent a to b.
auto Transport(const Vec3<Real> &a, const Vec3<Real> &b, const Vec3<Real> &u)
    -> Vec3<Real> {
  const Vec3<Real> axis = a.cross(b);
  const Real s = axis.norm();
  if (s < 1e-15) {
    return u;
  }
  return Eigen::AngleAxisd(std::atan2(s, a.dot(b)), axis / s) * u;
}

// Change of the last edge's Bishop angle when the centerline moves from x to
// y, with u0 carried in time on edge 0 and in space along each centerline.
auto Holonomy(const Mat<Real> &x, const Mat<Real> &y, const Vec3<Real> &u0)
    -> Real {
  const auto toEnd = [](const Mat<Real> &z, Vec3<Real> u) {
    for (int i = 1; i + 1 < z.rows(); ++i) {
      u = Transport(Edge(z, i - 1).normalized(), Edge(z, i).normalized(), u);
    }
    return u;
  };
  const int last = x.rows() - 2;
  const Vec3<Real> tEnd = Edge(y, last).normalized();
  const Vec3<Real> carried =
      Transport(Edge(x, last).normalized(), tEnd, toEnd(x, u0));
  const Vec3<Real> transported = toEnd(
      y, Transport(Edge(x, 0).normalized(), Edge(y, 0).normalized(), u0));
  return std::atan2(tEnd.dot(carried.cross(transported)),
                    carried.dot(transported));
}

// The per-step update sequence of ForwardEulerStrand::Step.
auto MakeRod(const Mat<Real> &x) -> DiscreteElasticRod {
  DiscreteElasticRod der(RestRod());
  der.vertices = x;
  der.UpdateLengths();
  der.Computekbs();
  der.UpdateBishopFrames();
  der.UpdateMaterialFrames();
  der.UpdateMaterialCurvatures();
  der.UpdateKbGradients();
  der.UpdateHolonomyGradient();
  return der;
}

auto Forces(DiscreteElasticRod &der, Real k, Real thetaEnd) -> Vec<Real> {
  der.springStiffness = k;
  der.thetas.setZero();
  der.thetas(der.thetas.rows() - 1) = thetaEnd;
  return der.ComputeCenterlineForcesStraight();
}

// sum_i alpha |kb_i|^2 / lbar_i with lbar_i = |e~^{i-1}| + |e~^i|.
auto BendEnergy(const Mat<Real> &x) -> Real {
  Real energy = 0;
  for (int i = 0; i + 2 < x.rows(); ++i) {
    energy += gBendingModulus * Kb(x, i).squaredNorm() /
              (kSpacing[i] + kSpacing[i + 1]);
  }
  return energy;
}

TEST(TestStrandForces, BendingForceIsMinusEnergyGradient) {
  const Mat<Real> x = StretchedRod();
  auto der = MakeRod(x);
  EXPECT_LE(RelErr(Forces(der, 0, 0), -FdGradient(BendEnergy, x)), 1e-6);
}

TEST(TestStrandForces, BendingForceOnFreeRodHasNoNetForce) {
  auto der = MakeRod(StretchedRod());
  const Vec<Real> bend = Forces(der, 0, 0);
  EXPECT_LE(bend.reshaped(3, bend.size() / 3).rowwise().sum().norm(), 1e-9);
}

// beta m^2 / L with m = theta^n - theta^0 minus the holonomy of the move.
TEST(TestStrandForces, TwistForceIsMinusEnergyGradient) {
  const Mat<Real> x = InextensibleRod();
  auto der = MakeRod(x);
  constexpr Real thetaEnd = 1;
  const Vec<Real> twist = Forces(der, 0, thetaEnd) - Forces(der, 0, 0);
  const Vec3<Real> u0 = OrthogonalVectorUnsafe(Edge(x, 0).normalized());
  const auto energy = [&](const Mat<Real> &y) {
    const Real m = thetaEnd - Holonomy(x, y, u0);
    return gTwistingModulus * m * m / der.totalRestLength;
  };
  EXPECT_LE(RelErr(twist, -FdGradient(energy, x)), 1e-6);
}

// The cubic spring -|e| grad Psi integrates to k (l^3/3 - lbar l^2/2), with
// lbar each edge's own rest length.
TEST(TestStrandForces, SpringForceIsMinusEnergyGradient) {
  const Mat<Real> x = StretchedRod();
  auto der = MakeRod(x);
  const Real k = der.springStiffness;
  const Vec<Real> spring = Forces(der, k, 0) - Forces(der, 0, 0);
  const auto energy = [&](const Mat<Real> &y) {
    Real e = 0;
    for (int i = 0; i + 1 < y.rows(); ++i) {
      const Real l = Edge(y, i).norm();
      e += k * (l * l * l / 3 - kSpacing[i] * l * l / 2);
    }
    return e;
  };
  EXPECT_LE(RelErr(spring, -FdGradient(energy, x)), 1e-6);
}

TEST(TestStrandForces, BishopFramesAreSpaceParallelTransported) {
  const Mat<Real> x = InextensibleRod();
  auto der = MakeRod(x);
  for (int i = 1; i < der.nRods; ++i) {
    const Vec3<Real> want = Transport(der.edges.at(i - 1).t, der.edges.at(i).t,
                                      der.frames.at(i - 1).u);
    EXPECT_LE((der.frames.at(i).u - want).norm(), 1e-12) << "edge " << i;
  }
}

TEST(TestStrandForces, FirstBishopFrameFollowsTheFirstEdge) {
  const Mat<Real> x = InextensibleRod();
  auto der = MakeRod(x);
  const Vec3<Real> want = Transport(Vec3<Real>::UnitX(), der.edges.at(0).t,
                                    OrthogonalVectorUnsafe(Vec3<Real>::UnitX()));
  EXPECT_LE((der.frames.at(0).u - want).norm(), 1e-12);
  EXPECT_LE((der.frames.at(0).t - der.edges.at(0).t).norm(), 1e-12);
}

} // namespace
