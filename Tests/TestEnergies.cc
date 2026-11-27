// Checks every volume Energy must pass. A new energy joins by adding its type
// to Energies and a HandDerived specialization.

#include <Energy/Volume/StVenantKirchhoff.h>
#include <Energy/Volume/StableNeoHookean.h>

// The energy StableNeoHookean replaces, kept until TetMesh moves over.
#include <Energy/Volume/SNH.h>

#include <gtest/gtest.h>
#include <random>

using namespace sim;

namespace {

// The bunny-drop material.
constexpr Real kYoungs = 5.0;
constexpr Real kPoisson = 0.45;
const Lame kLame = Lame::FromYoungs(kYoungs, kPoisson);

// 1,000 deformation gradients: half near rest, half with entries in
// [-1.5, 1.5], about half of which are inverted.
auto SampleFs() -> const std::vector<Mat3<Real>> & {
  static const std::vector<Mat3<Real>> fs = [] {
    std::mt19937 rng(7);
    std::uniform_real_distribution<Real> small(-0.5, 0.5);
    std::uniform_real_distribution<Real> large(-1.5, 1.5);
    std::vector<Mat3<Real>> out;
    for (int n = 0; n < 1000; ++n) {
      Mat3<Real> f;
      for (int k = 0; k < 9; ++k)
        f(k) = n % 2 == 0 ? small(rng) : large(rng);
      if (n % 2 == 0)
        f += Mat3<Real>::Identity();
      out.push_back(f);
    }
    return out;
  }();
  return fs;
}

auto RandomRotation(std::mt19937 &rng) -> Mat3<Real> {
  std::normal_distribution<Real> normal;
  return Eigen::Quaternion<Real>(normal(rng), normal(rng), normal(rng),
                                 normal(rng))
      .normalized()
      .toRotationMatrix();
}

template <typename A, typename B>
auto RelErr(const A &got, const B &want) -> Real {
  return (got - want).norm() / std::max(got.norm(), want.norm());
}

constexpr Real kFdStep = 1e-6;

// Central differences of Psi in each entry of F.
template <typename E> auto FdPk1(const E &e, const Mat3<Real> &f) -> Mat3<Real> {
  Mat3<Real> out;
  for (int k = 0; k < 9; ++k) {
    Mat3<Real> plus = f;
    Mat3<Real> minus = f;
    plus(k) += kFdStep;
    minus(k) -= kFdStep;
    out(k) = (e.Psi(plus) - e.Psi(minus)) / (2 * kFdStep);
  }
  return out;
}

// Column k is the central difference of vec(P) in vec(F)(k).
template <typename E>
auto FdHessian(const E &e, const Mat3<Real> &f) -> Mat9<Real> {
  Mat9<Real> out;
  for (int k = 0; k < 9; ++k) {
    Mat3<Real> plus = f;
    Mat3<Real> minus = f;
    plus(k) += kFdStep;
    minus(k) -= kFdStep;
    out.col(k) = Vectorize(e.Pk1(plus) - e.Pk1(minus)) / (2 * kFdStep);
  }
  return out;
}

// The PSD-cone projection of h, computed numerically.
auto ProjectPsd(const Mat9<Real> &h) -> Mat9<Real> {
  const Eigen::SelfAdjointEigenSolver<Mat9<Real>> eig(h);
  return eig.eigenvectors() *
         eig.eigenvalues().cwiseMax(0).asDiagonal() *
         eig.eigenvectors().transpose();
}

// One F worked out by hand per energy.
template <typename E> struct HandDerived;

// F = diag(2, 1, 1), mu = 1, lambda = 2, so the SNH lambda is l = 3.
// |F|^2 = 6 and J = 2: Psi = 1/2 (6 - 3) - (2 - 1) + 3/2 (2 - 1)^2 = 2.
// dJ/dF = cof F = diag(1, 2, 2) and l (J - 1) - mu = 2:
// P = diag(2, 1, 1) + 2 diag(1, 2, 2) = diag(4, 5, 5).
template <> struct HandDerived<StableNeoHookean> {
  static constexpr Real psi = 2;
  static auto Pk1() -> Mat3<Real> { return Vec3<Real>(4, 5, 5).asDiagonal(); }
};

// F = diag(2, 1, 1), mu = 1, lambda = 2. E = diag(3/2, 0, 0), |E|^2 = 9/4 and
// tr E = 3/2: Psi = 9/4 + 9/4 = 9/2. S = 2 E + 2 (3/2) I = diag(6, 3, 3):
// P = F S = diag(12, 3, 3).
template <> struct HandDerived<StVenantKirchhoff> {
  static constexpr Real psi = 4.5;
  static auto Pk1() -> Mat3<Real> { return Vec3<Real>(12, 3, 3).asDiagonal(); }
};

template <typename E> class EnergyTest : public testing::Test {};

using Energies = testing::Types<StableNeoHookean, StVenantKirchhoff>;
TYPED_TEST_SUITE(EnergyTest, Energies);

TEST(EnergySamples, IncludeInvertedF) {
  int inverted = 0;
  for (const Mat3<Real> &f : SampleFs())
    inverted += f.determinant() < 0;
  EXPECT_GE(inverted, 200);
}

TYPED_TEST(EnergyTest, Pk1MatchesFdOfPsi) {
  const TypeParam e{kLame};
  Real worst = 0;
  std::size_t worstAt = 0;
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Real err = RelErr(e.Pk1(f), FdPk1(e, f));
    if (err > worst) {
      worst = err;
      worstAt = n;
    }
  }
  EXPECT_LE(worst, 1e-6) << "sample " << worstAt;
}

TYPED_TEST(EnergyTest, HessianMatchesFdOfPk1) {
  const TypeParam e{kLame};
  Real worst = 0;
  std::size_t worstAt = 0;
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Real err = RelErr(e.Hessian(f), FdHessian(e, f));
    if (err > worst) {
      worst = err;
      worstAt = n;
    }
  }
  EXPECT_LE(worst, 1e-6) << "sample " << worstAt;
}

TYPED_TEST(EnergyTest, ClampedHessianIsPsd) {
  const TypeParam e{kLame};
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Mat9<Real> h = e.ClampedHessian(f, RotationVariantSvd(f));
    const Real minEigenvalue =
        Eigen::SelfAdjointEigenSolver<Mat9<Real>>(h).eigenvalues().minCoeff();
    ASSERT_GE(minEigenvalue, -1e-10 * h.norm()) << "sample " << n;
  }
}

// PSD alone passes for a zero matrix. The analytic eigensystem must give the
// same projection as an eigendecomposition of the exact Hessian.
TYPED_TEST(EnergyTest, ClampedHessianIsPsdProjectionOfHessian) {
  const TypeParam e{kLame};
  Real worst = 0;
  std::size_t worstAt = 0;
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Real err = RelErr(e.ClampedHessian(f, RotationVariantSvd(f)),
                            ProjectPsd(e.Hessian(f)));
    if (err > worst) {
      worst = err;
      worstAt = n;
    }
  }
  EXPECT_LE(worst, 1e-9) << "sample " << worstAt;
}

TYPED_TEST(EnergyTest, RotationsAreAtRest) {
  const TypeParam e{kLame};
  const Real scale = kLame.mu + kLame.lambda;
  std::mt19937 rng(8);
  for (int n = 0; n < 100; ++n) {
    const Mat3<Real> r = RandomRotation(rng);
    EXPECT_LE(std::abs(e.Psi(r)), 1e-12 * scale) << "rotation " << n;
    EXPECT_LE(e.Pk1(r).norm(), 1e-12 * scale) << "rotation " << n;
  }
}

TYPED_TEST(EnergyTest, PsiIsRotationInvariant) {
  const TypeParam e{kLame};
  std::mt19937 rng(9);
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Mat3<Real> r = RandomRotation(rng);
    const Real psi = e.Psi(f);
    EXPECT_NEAR(e.Psi(r * f), psi, 1e-12 * std::abs(psi)) << "sample " << n;
    EXPECT_NEAR(e.Psi(f * r), psi, 1e-12 * std::abs(psi)) << "sample " << n;
  }
}

TYPED_TEST(EnergyTest, MatchesHandDerivedValue) {
  const TypeParam e{Lame{.mu = 1, .lambda = 2}};
  const Mat3<Real> f = Vec3<Real>(2, 1, 1).asDiagonal();
  EXPECT_NEAR(e.Psi(f), HandDerived<TypeParam>::psi, 1e-14);
  EXPECT_LE((e.Pk1(f) - HandDerived<TypeParam>::Pk1()).norm(), 1e-14);
}

TEST(RotationVariantSvd, ReconstructsWithRotations) {
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Svd svd = RotationVariantSvd(f);
    const Mat3<Real> back = svd.u * svd.sigma.asDiagonal() * svd.v.transpose();
    EXPECT_LE((back - f).norm(), 1e-12 * f.norm()) << "sample " << n;
    EXPECT_NEAR(svd.u.determinant(), 1, 1e-12) << "sample " << n;
    EXPECT_NEAR(svd.v.determinant(), 1, 1e-12) << "sample " << n;
    EXPECT_EQ(svd.sigma(2) < 0, f.determinant() < 0) << "sample " << n;
  }
}

// H1b moves TetMesh to StableNeoHookean and keeps the bunny golden, so the
// forces must match SNH. SNH::Psi carries the constant mu^2 / (2 l) because
// it writes l/2 (J - 1)^2 - mu (J - 1) as l/2 (J - 1 - mu/l)^2.
TEST(StableNeoHookean, MatchesLegacySnh) {
  const SNH legacy(kYoungs, kPoisson);
  const StableNeoHookean snh{kLame};
  const Real l = kLame.lambda + kLame.mu;
  const Real offset = kLame.mu * kLame.mu / (2 * l);
  for (std::size_t n = 0; n < SampleFs().size(); ++n) {
    const Mat3<Real> &f = SampleFs()[n];
    const Real want = legacy.Psi(f);
    EXPECT_NEAR(snh.Psi(f) + offset, want, 1e-14 * std::abs(want))
        << "sample " << n;
    EXPECT_LE(RelErr(snh.Pk1(f), Mat3<Real>(legacy.Pk1(f))), 1e-14)
        << "sample " << n;
  }
}

} // namespace
