// Checks every ElementEnergy must pass: the gradient and Hessian match finite
// differences of the level below, and the clamped Hessian is PSD.

#include <Energy/Collision/VertexFaceSqrtCollision.h>
#include <Energy/Spring/MassSpring.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <random>

using namespace sim;

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

// A vertex in contact with a random triangle: in front of or behind its face,
// beside one of its edges, or off one of its corners, at a distance of
// `stretch` / 2.2 times epsilon, so inside the contact envelope for every
// stretch the suite draws. Each draw keeps clear of the boundaries where Psi
// is not smooth: the triangle's plane and the face, edge and corner regions.
template <> struct Sample<VertexFaceSqrtCollision> {
  static constexpr int N = 12;

  static auto Draw(std::mt19937 &rng, Real stretch)
      -> std::pair<VertexFaceSqrtCollision, VecN<N>> {
    std::uniform_real_distribution<Real> unit(-1, 1);
    std::uniform_real_distribution<Real> fraction(0, 1);
    const VertexFaceSqrtCollision e(3, 0.1);
    const Real distance = e.collisionEpsilon * stretch / 2.2;

    std::array<Vec3<Real>, 3> tri;
    Vec3<Real> n;
    do {
      for (auto &corner : tri) {
        corner = Vec3<Real>(unit(rng), unit(rng), unit(rng));
      }
      n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    } while (n.norm() < 0.5 || MinAngle(tri) < 0.4);
    n.normalize();

    Vec3<Real> vertex;
    switch (std::uniform_int_distribution<int>(0, 2)(rng)) {
    case 0: { // face
      Vec3<Real> b;
      do {
        b << fraction(rng), fraction(rng), 0;
        b(2) = 1 - b(0) - b(1);
      } while (b.minCoeff() < 0.15);
      const Real side = fraction(rng) < 0.5 ? -1 : 1;
      vertex = b(0) * tri[0] + b(1) * tri[1] + b(2) * tri[2] +
               side * distance * n;
      break;
    }
    case 1: { // edge
      const int i = std::uniform_int_distribution<int>(0, 2)(rng);
      const Vec3<Real> &a = tri[i];
      const Vec3<Real> &b = tri[(i + 1) % 3];
      const Vec3<Real> &opposite = tri[(i + 2) % 3];
      const Vec3<Real> along = (b - a).normalized();
      Vec3<Real> out = n.cross(along);
      if (out.dot(opposite - a) > 0) {
        out = -out;
      }
      const Real angle = std::uniform_real_distribution<Real>(0.3, 1.2)(rng) *
                         (fraction(rng) < 0.5 ? -1 : 1);
      const Vec3<Real> onEdge = a + (0.2 + 0.6 * fraction(rng)) * (b - a);
      vertex = onEdge + distance * (std::cos(angle) * out +
                                    std::sin(angle) * n);
      break;
    }
    default: { // corner
      const int i = std::uniform_int_distribution<int>(0, 2)(rng);
      const Vec3<Real> toB = (tri[(i + 1) % 3] - tri[i]).normalized();
      const Vec3<Real> toC = (tri[(i + 2) % 3] - tri[i]).normalized();
      Vec3<Real> dir;
      do {
        dir = Vec3<Real>(unit(rng), unit(rng), unit(rng));
      } while (dir.norm() > 1 || dir.norm() < 0.1 ||
               dir.normalized().dot(toB) > -0.2 ||
               dir.normalized().dot(toC) > -0.2 ||
               std::abs(dir.normalized().dot(n)) < 0.2);
      vertex = tri[i] + distance * dir.normalized();
      break;
    }
    }
    VecN<N> x;
    x << vertex, tri[0], tri[1], tri[2];
    return {e, x};
  }

  // The smallest interior angle, in radians.
  static auto MinAngle(const std::array<Vec3<Real>, 3> &tri) -> Real {
    Real smallest = 4;
    for (int i = 0; i < 3; ++i) {
      const Vec3<Real> u = (tri[(i + 1) % 3] - tri[i]).normalized();
      const Vec3<Real> w = (tri[(i + 2) % 3] - tri[i]).normalized();
      smallest = std::min(smallest, std::acos(std::clamp<Real>(u.dot(w), -1, 1)));
    }
    return smallest;
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

using Energies = testing::Types<MassSpring, VertexFaceSqrtCollision>;
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

// Many random contacts across the face, edge and corner regions, on both
// sides of the triangle.
TEST(VertexFaceSqrtCollisionTest, DerivativesMatchFdAcrossContacts) {
  using S = Sample<VertexFaceSqrtCollision>;
  std::mt19937 rng(5);
  std::uniform_real_distribution<Real> stretch(0.1, 2);
  for (int i = 0; i < 300; ++i) {
    const auto [e, x] = S::Draw(rng, stretch(rng));
    const auto psi = [&](const VecN<12> &y) {
      return Eigen::Matrix<Real, 1, 1>(e.Psi(y));
    };
    const auto gradient = [&](const VecN<12> &y) { return e.Gradient(y); };
    EXPECT_LE(RelErr(e.Gradient(x), FdJacobian<12>(psi, x).transpose()), 1e-6)
        << "sample " << i;
    EXPECT_LE(RelErr(e.Hessian(x), FdJacobian<12>(gradient, x)), 1e-6)
        << "sample " << i;
  }
}

TEST(VertexFaceSqrtCollisionTest, ClampedHessianIsEigenProjection) {
  using S = Sample<VertexFaceSqrtCollision>;
  std::mt19937 rng(6);
  std::uniform_real_distribution<Real> stretch(0.1, 2);
  for (int i = 0; i < 100; ++i) {
    const auto [e, x] = S::Draw(rng, stretch(rng));
    const Eigen::SelfAdjointEigenSolver<MatN<12>> eigen(e.Hessian(x));
    const MatN<12> projected =
        eigen.eigenvectors() *
        eigen.eigenvalues().cwiseMax(0).asDiagonal() *
        eigen.eigenvectors().transpose();
    EXPECT_LE(RelErr(e.ClampedHessian(x), projected), 1e-12) << "sample " << i;
  }
}

} // namespace
