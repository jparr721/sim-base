// Checks every volume Energy must pass. A new energy joins by adding its type
// to Energies and a HandDerived specialization, plus a Traits specialization
// when it is not isotropic or not defined for every F.

#include <Energy/Volume/LinearElasticity.h>
#include <Energy/Volume/NeoHookeanBonetWood.h>
#include <Energy/Volume/StVenantKirchhoff.h>
#include <Energy/Volume/StableNeoHookean.h>

#include <gtest/gtest.h>
#include <random>

using namespace sim;

namespace {

// The bunny-drop material.
constexpr Real kYoungs = 5.0;
constexpr Real kPoisson = 0.45;
const Lame kLame = Lame::FromYoungs(kYoungs, kPoisson);

// Even draws are near rest. Odd draws have entries in [-1.5, 1.5], and about
// half of them are inverted.
auto DrawF(std::mt19937 &rng, int n) -> Mat3<Real> {
  std::uniform_real_distribution<Real> small(-0.5, 0.5);
  std::uniform_real_distribution<Real> large(-1.5, 1.5);
  Mat3<Real> f;
  for (int k = 0; k < 9; ++k)
    f(k) = n % 2 == 0 ? small(rng) : large(rng);
  if (n % 2 == 0)
    f += Mat3<Real>::Identity();
  return f;
}

// The first 1,000 draws that satisfy inDomain.
auto SampleFsWhere(bool (*inDomain)(const Mat3<Real> &))
    -> std::vector<Mat3<Real>> {
  std::mt19937 rng(7);
  std::vector<Mat3<Real>> out;
  for (int n = 0; out.size() < 1000; ++n) {
    const Mat3<Real> f = DrawF(rng, n);
    if (inDomain(f))
      out.push_back(f);
  }
  return out;
}

auto AnyF(const Mat3<Real> & /*f*/) -> bool { return true; }

auto SampleFs() -> const std::vector<Mat3<Real>> & {
  static const std::vector<Mat3<Real>> fs = SampleFsWhere(AnyF);
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

// What the suite asserts about an energy's symmetry.
//  kIsotropic: Psi(R F) = Psi(F R) = Psi(F) for every rotation R, and every
//    rotation is at rest.
//  kNone: only F = I is at rest. Rotations are strained and Psi(R F) differs
//    from Psi(F), and the suite checks both so the trait is proven.
enum class Symmetry { kIsotropic, kNone };

// How the suite builds an energy and where it samples F. An energy that is
// singular somewhere samples only the part of its domain kept away from the
// singularity, so finite differences stay meaningful.
// A specialization derives from DefaultTraits and states only what differs.
template <typename E> struct DefaultTraits {
  static constexpr Symmetry symmetry = Symmetry::kIsotropic;
  static auto Make(const Lame &lame) -> E { return E{lame}; }
  static auto InDomain(const Mat3<Real> &f) -> bool { return AnyF(f); }
};
template <typename E> struct Traits : DefaultTraits<E> {};

template <typename E> auto DomainFs() -> const std::vector<Mat3<Real>> & {
  static const std::vector<Mat3<Real>> fs = SampleFsWhere(Traits<E>::InDomain);
  return fs;
}

// One F worked out by hand per energy. Unless a specialization says
// otherwise, F = diag(2, 1, 1), mu = 1 and lambda = 2.
template <typename E> struct HandDerivedInput {
  static auto Make() -> E { return Traits<E>::Make({.mu = 1, .lambda = 2}); }
  static auto F() -> Mat3<Real> { return Vec3<Real>(2, 1, 1).asDiagonal(); }
};
template <typename E> struct HandDerived;

template <>
struct Traits<LinearElasticity> : DefaultTraits<LinearElasticity> {
  static constexpr Symmetry symmetry = Symmetry::kNone;
};

// l = lambda + mu = 3. |F|^2 = 6 and J = 2:
// Psi = 1/2 (6 - 3) - (2 - 1) + 3/2 (2 - 1)^2 = 2.
// dJ/dF = cof F = diag(1, 2, 2) and l (J - 1) - mu = 2:
// P = diag(2, 1, 1) + 2 diag(1, 2, 2) = diag(4, 5, 5).
template <>
struct HandDerived<StableNeoHookean> : HandDerivedInput<StableNeoHookean> {
  static auto Psi() -> Real { return 2; }
  static auto Pk1() -> Mat3<Real> { return Vec3<Real>(4, 5, 5).asDiagonal(); }
};

// E = diag(3/2, 0, 0), |E|^2 = 9/4 and tr E = 3/2: Psi = 9/4 + 9/4 = 9/2.
// S = 2 E + 2 (3/2) I = diag(6, 3, 3): P = F S = diag(12, 3, 3).
template <>
struct HandDerived<StVenantKirchhoff> : HandDerivedInput<StVenantKirchhoff> {
  static auto Psi() -> Real { return 4.5; }
  static auto Pk1() -> Mat3<Real> { return Vec3<Real>(12, 3, 3).asDiagonal(); }
};

// eps = diag(1, 0, 0): Psi = 1 + 2/2 (1)^2 = 2.
// P = 2 eps + 2 tr(eps) I = diag(4, 2, 2).
template <>
struct HandDerived<LinearElasticity> : HandDerivedInput<LinearElasticity> {
  static auto Psi() -> Real { return 2; }
  static auto Pk1() -> Mat3<Real> { return Vec3<Real>(4, 2, 2).asDiagonal(); }
};

// J = 2 >= kGuardJ, so log J is exact. |F|^2 = 6, F^-T = diag(1/2, 1, 1):
// Psi = 1/2 (6 - 3) - log 2 + 2/2 (log 2)^2 = 3/2 - log 2 + (log 2)^2.
// P = (F - F^-T) + 2 log 2 F^-T = diag(3/2 + log 2, 2 log 2, 2 log 2).
template <>
struct HandDerived<NeoHookeanBonetWood>
    : HandDerivedInput<NeoHookeanBonetWood> {
  static auto Psi() -> Real {
    return Real(1.5) - std::log(Real(2)) + std::pow(std::log(Real(2)), 2);
  }
  static auto Pk1() -> Mat3<Real> {
    const Real ln2 = std::log(Real(2));
    return Vec3<Real>(Real(1.5) + ln2, 2 * ln2, 2 * ln2).asDiagonal();
  }
};

template <typename E> class EnergyTest : public testing::Test {};

using Energies =
    testing::Types<StableNeoHookean,
                   StVenantKirchhoff,
                   LinearElasticity,
                   NeoHookeanBonetWood>;
TYPED_TEST_SUITE(EnergyTest, Energies);

TEST(EnergySamples, IncludeInvertedF) {
  int inverted = 0;
  for (const Mat3<Real> &f : SampleFs())
    inverted += f.determinant() < 0;
  EXPECT_GE(inverted, 200);
}

TYPED_TEST(EnergyTest, Pk1MatchesFdOfPsi) {
  const TypeParam e = Traits<TypeParam>::Make(kLame);
  const std::vector<Mat3<Real>> &fs = DomainFs<TypeParam>();
  Real worst = 0;
  std::size_t worstAt = 0;
  for (std::size_t n = 0; n < fs.size(); ++n) {
    const Real err = RelErr(e.Pk1(fs[n]), FdPk1(e, fs[n]));
    if (err > worst) {
      worst = err;
      worstAt = n;
    }
  }
  EXPECT_LE(worst, 1e-6) << "sample " << worstAt;
}

TYPED_TEST(EnergyTest, HessianMatchesFdOfPk1) {
  const TypeParam e = Traits<TypeParam>::Make(kLame);
  const std::vector<Mat3<Real>> &fs = DomainFs<TypeParam>();
  Real worst = 0;
  std::size_t worstAt = 0;
  for (std::size_t n = 0; n < fs.size(); ++n) {
    const Real err = RelErr(e.Hessian(fs[n]), FdHessian(e, fs[n]));
    if (err > worst) {
      worst = err;
      worstAt = n;
    }
  }
  EXPECT_LE(worst, 1e-6) << "sample " << worstAt;
}

TYPED_TEST(EnergyTest, ClampedHessianIsPsd) {
  const TypeParam e = Traits<TypeParam>::Make(kLame);
  const std::vector<Mat3<Real>> &fs = DomainFs<TypeParam>();
  for (std::size_t n = 0; n < fs.size(); ++n) {
    const Mat9<Real> h = e.ClampedHessian(fs[n], RotationVariantSvd(fs[n]));
    const Real minEigenvalue =
        Eigen::SelfAdjointEigenSolver<Mat9<Real>>(h).eigenvalues().minCoeff();
    ASSERT_GE(minEigenvalue, -1e-10 * h.norm()) << "sample " << n;
  }
}

// PSD alone passes for a zero matrix. The analytic eigensystem must give the
// same projection as an eigendecomposition of the exact Hessian.
TYPED_TEST(EnergyTest, ClampedHessianIsPsdProjectionOfHessian) {
  const TypeParam e = Traits<TypeParam>::Make(kLame);
  const std::vector<Mat3<Real>> &fs = DomainFs<TypeParam>();
  Real worst = 0;
  std::size_t worstAt = 0;
  for (std::size_t n = 0; n < fs.size(); ++n) {
    const Real err = RelErr(e.ClampedHessian(fs[n], RotationVariantSvd(fs[n])),
                            ProjectPsd(e.Hessian(fs[n])));
    if (err > worst) {
      worst = err;
      worstAt = n;
    }
  }
  EXPECT_LE(worst, 1e-9) << "sample " << worstAt;
}

TYPED_TEST(EnergyTest, RestStatesMatchSymmetry) {
  const TypeParam e = Traits<TypeParam>::Make(kLame);
  const Real scale = kLame.mu + kLame.lambda;
  const Mat3<Real> identity = Mat3<Real>::Identity();
  EXPECT_LE(std::abs(e.Psi(identity)), 1e-12 * scale);
  EXPECT_LE(e.Pk1(identity).norm(), 1e-12 * scale);
  std::mt19937 rng(8);
  for (int n = 0; n < 100; ++n) {
    const Mat3<Real> r = RandomRotation(rng);
    if constexpr (Traits<TypeParam>::symmetry == Symmetry::kNone) {
      EXPECT_GT(e.Psi(r), 1e-12 * scale) << "rotation " << n;
    } else {
      EXPECT_LE(std::abs(e.Psi(r)), 1e-12 * scale) << "rotation " << n;
      EXPECT_LE(e.Pk1(r).norm(), 1e-12 * scale) << "rotation " << n;
    }
  }
}

TYPED_TEST(EnergyTest, PsiHasItsSymmetry) {
  const TypeParam e = Traits<TypeParam>::Make(kLame);
  const std::vector<Mat3<Real>> &fs = DomainFs<TypeParam>();
  std::mt19937 rng(9);
  int changed = 0;
  for (std::size_t n = 0; n < fs.size(); ++n) {
    const Mat3<Real> &f = fs[n];
    const Mat3<Real> r = RandomRotation(rng);
    const Real psi = e.Psi(f);
    const auto same = [&](const Mat3<Real> &g) {
      return std::abs(e.Psi(g) - psi) <= 1e-12 * std::abs(psi);
    };
    if constexpr (Traits<TypeParam>::symmetry == Symmetry::kNone) {
      changed += !same(r * f);
    } else {
      EXPECT_TRUE(same(r * f)) << "sample " << n;
      EXPECT_TRUE(same(f * r)) << "sample " << n;
    }
  }
  if constexpr (Traits<TypeParam>::symmetry == Symmetry::kNone)
    EXPECT_GE(changed, 900);
}

TYPED_TEST(EnergyTest, MatchesHandDerivedValue) {
  using Hand = HandDerived<TypeParam>;
  const TypeParam e = Hand::Make();
  EXPECT_NEAR(e.Psi(Hand::F()), Hand::Psi(), 1e-14);
  EXPECT_LE((e.Pk1(Hand::F()) - Hand::Pk1()).norm(), 1e-14);
}

// F = diag(1, 1, t) as t falls from 1 through the guard at t = kGuardJ, to 0
// and into inversion: Psi, P and the Hessian stay finite, Psi keeps growing,
// and Psi and P are continuous across the guard.
TEST(NeoHookeanBonetWood, GuardIsFiniteAndGrowsAsJShrinks) {
  const NeoHookeanBonetWood e{kLame};
  const auto at = [](Real t) -> Mat3<Real> {
    return Vec3<Real>(1, 1, t).asDiagonal();
  };
  Real previous = e.Psi(at(1));
  for (const Real t : {0.5, 0.2, 0.11, 0.1, 0.09, 0.05, 1e-2, 1e-4, 1e-8, 0.0,
                       -1e-8, -0.1, -1.0}) {
    const Real psi = e.Psi(at(t));
    EXPECT_TRUE(std::isfinite(psi)) << "t = " << t;
    EXPECT_TRUE(e.Pk1(at(t)).allFinite()) << "t = " << t;
    EXPECT_TRUE(e.Hessian(at(t)).allFinite()) << "t = " << t;
    EXPECT_GT(psi, previous) << "t = " << t;
    previous = psi;
  }

  // A step of 2 dt across the guard changes Psi and P by no more than their
  // slopes allow, so neither jumps there.
  constexpr Real g = NeoHookeanBonetWood::kGuardJ;
  constexpr Real dt = 1e-9;
  const Real slack = 2 * dt * 1.01;
  EXPECT_LE(std::abs(e.Psi(at(g - dt)) - e.Psi(at(g + dt))),
            slack * e.Pk1(at(g)).norm());
  EXPECT_LE((e.Pk1(at(g - dt)) - e.Pk1(at(g + dt))).norm(),
            slack * e.Hessian(at(g)).norm());
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

} // namespace
