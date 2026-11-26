// Checks every ElementEnergy must pass: the gradient and Hessian match finite
// differences of the level below, and the clamped Hessian is PSD.

#include <Energy/Spring/MassSpring.h>

#include <gtest/gtest.h>
#include <random>

namespace {

// How a typed test draws one random element and its DOFs.
template <typename E> struct Sample;

template <> struct Sample<MassSpring> {
  static constexpr int N = 6;

  // An edge with a random direction whose length is `stretch` times its rest
  // length.
  static auto Draw(std::mt19937 &rng, Real stretch)
      -> std::pair<MassSpring, VecN<N>> {
    std::uniform_real_distribution<Real> unit(-1, 1);
    std::uniform_real_distribution<Real> rest(0.5, 2);
    const Vec3<Real> x0(unit(rng), unit(rng), unit(rng));
    const Vec3<Real> dir =
        Vec3<Real>(unit(rng), unit(rng), unit(rng)).normalized();
    const MassSpring spring{.k = 3, .restLength = rest(rng)};
    VecN<N> x;
    x << x0, x0 + stretch * spring.restLength * dir;
    return {spring, x};
  }
};

template <int N, typename F>
auto FdJacobian(const F &f, const VecN<N> &x) {
  constexpr Real h = 1e-6;
  using Column = decltype(f(x));
  Eigen::Matrix<Real, Column::RowsAtCompileTime, N> out;
  for (int i = 0; i < N; ++i) {
    VecN<N> plus = x;
    VecN<N> minus = x;
    plus(i) += h;
    minus(i) -= h;
    out.col(i) = (f(plus) - f(minus)) / (2 * h);
  }
  return out;
}

template <typename A, typename B>
auto RelErr(const A &got, const B &want) -> Real {
  return (got - want).norm() / std::max(got.norm(), want.norm());
}

template <int N> auto MinEigenvalue(const MatN<N> &h) -> Real {
  return Eigen::SelfAdjointEigenSolver<MatN<N>>(h).eigenvalues().minCoeff();
}

constexpr Real kStretched = 1.4;
constexpr Real kCompressed = 0.6;

template <typename E> class ElementEnergyTest : public testing::Test {};

using Energies = testing::Types<MassSpring>;
TYPED_TEST_SUITE(ElementEnergyTest, Energies);

TYPED_TEST(ElementEnergyTest, GradientMatchesFdOfPsi) {
  constexpr int N = Sample<TypeParam>::N;
  std::mt19937 rng(1);
  for (const Real stretch : {kStretched, kCompressed}) {
    const auto [e, x] = Sample<TypeParam>::Draw(rng, stretch);
    const auto psi = [&](const VecN<N> &y) {
      return Eigen::Matrix<Real, 1, 1>(e.Psi(y));
    };
    EXPECT_LE(RelErr(e.Gradient(x), FdJacobian<N>(psi, x).transpose()), 1e-6)
        << "stretch " << stretch;
  }
}

TYPED_TEST(ElementEnergyTest, HessianMatchesFdOfGradient) {
  constexpr int N = Sample<TypeParam>::N;
  std::mt19937 rng(2);
  for (const Real stretch : {kStretched, kCompressed}) {
    const auto [e, x] = Sample<TypeParam>::Draw(rng, stretch);
    const auto gradient = [&](const VecN<N> &y) { return e.Gradient(y); };
    EXPECT_LE(RelErr(e.Hessian(x), FdJacobian<N>(gradient, x)), 1e-6)
        << "stretch " << stretch;
  }
}

TYPED_TEST(ElementEnergyTest, ClampedHessianIsPsd) {
  constexpr int N = Sample<TypeParam>::N;
  std::mt19937 rng(3);
  std::uniform_real_distribution<Real> stretch(0.1, 2);
  for (int i = 0; i < 1000; ++i) {
    const auto [e, x] = Sample<TypeParam>::Draw(rng, stretch(rng));
    EXPECT_GE(MinEigenvalue<N>(e.ClampedHessian(x)), -1e-10) << "sample " << i;
  }
}

// The control: without clamping, a compressed element is indefinite.
TYPED_TEST(ElementEnergyTest, HessianIsIndefiniteUnderCompression) {
  constexpr int N = Sample<TypeParam>::N;
  std::mt19937 rng(4);
  const auto [e, x] = Sample<TypeParam>::Draw(rng, kCompressed);
  EXPECT_LT(MinEigenvalue<N>(e.Hessian(x)), -1e-3);
}

} // namespace
