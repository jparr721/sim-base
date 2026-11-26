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
  der.massSpring->k = k;
  der.thetas.setZero();
  der.thetas(der.thetas.rows() - 1) = thetaEnd;
  return der.ComputeCenterlineForcesStraight();
}

// sum_i beta |kb_i|^2 / |e~^0|: the library's bending stiffness, kept as is.
auto BendEnergy(const Mat<Real> &x) -> Real {
  Real energy = 0;
  for (int i = 0; i + 2 < x.rows(); ++i) {
    energy += gTwistingModulus * Kb(x, i).squaredNorm() / kSpacing[0];
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

} // namespace
