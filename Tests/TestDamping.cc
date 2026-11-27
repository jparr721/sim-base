// Checks every DampingEnergy must pass, on GreenDamping.

#include <Energy/Damping/GreenDamping.h>

#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <random>
#include <utility>
#include <vector>

using namespace sim;

namespace {

const GreenDamping kDamping{.mu = 0.7};

constexpr Real kFdStep = 1e-6;

// 1,000 seeded (F, Fdot). Even draws have F near rest, odd draws have entries
// in [-1.5, 1.5], so about half of those are inverted.
auto Samples() -> const std::vector<std::pair<Mat3<Real>, Mat3<Real>>> & {
  static const auto samples = [] {
    std::mt19937 rng(13);
    std::uniform_real_distribution<Real> small(-0.5, 0.5);
    std::uniform_real_distribution<Real> large(-1.5, 1.5);
    std::uniform_real_distribution<Real> rate(-2, 2);
    std::vector<std::pair<Mat3<Real>, Mat3<Real>>> out;
    for (int n = 0; n < 1000; ++n) {
      Mat3<Real> f;
      Mat3<Real> fdot;
      for (int k = 0; k < 9; ++k) {
        f(k) = n % 2 == 0 ? small(rng) : large(rng);
        fdot(k) = rate(rng);
      }
      if (n % 2 == 0)
        f += Mat3<Real>::Identity();
      out.emplace_back(f, fdot);
    }
    return out;
  }();
  return samples;
}

template <typename A, typename B>
auto RelErr(const A &got, const B &want) -> Real {
  return (got - want).norm() / std::max(got.norm(), want.norm());
}

TEST(GreenDamping, Pk1IsTheRateGradientOfPsi) {
  Real worst = 0;
  for (const auto &[f, fdot] : Samples()) {
    Mat3<Real> fd;
    for (int k = 0; k < 9; ++k) {
      Mat3<Real> plus = fdot;
      Mat3<Real> minus = fdot;
      plus(k) += kFdStep;
      minus(k) -= kFdStep;
      fd(k) = (kDamping.Psi(f, plus) - kDamping.Psi(f, minus)) / (2 * kFdStep);
    }
    worst = std::max(worst, RelErr(kDamping.Pk1(f, fdot), fd));
  }
  EXPECT_LE(worst, 1e-6);
}

TEST(GreenDamping, HessianIsTheRateJacobianOfPk1) {
  Real worst = 0;
  for (const auto &[f, fdot] : Samples()) {
    Mat9<Real> fd;
    for (int k = 0; k < 9; ++k) {
      Mat3<Real> plus = fdot;
      Mat3<Real> minus = fdot;
      plus(k) += kFdStep;
      minus(k) -= kFdStep;
      fd.col(k) = Vectorize(kDamping.Pk1(f, plus) - kDamping.Pk1(f, minus)) /
                  (2 * kFdStep);
    }
    worst = std::max(worst, RelErr(kDamping.Hessian(f, fdot), fd));
  }
  EXPECT_LE(worst, 1e-6);
}

// Psi is the quadratic form of its Hessian, Psi = vec(Fdot)^T H vec(Fdot) / 2,
// and H has no negative eigenvalue beyond round-off.
TEST(GreenDamping, HessianIsPsdAndPsiIsItsQuadraticForm) {
  for (const auto &[f, fdot] : Samples()) {
    const Mat9<Real> h = kDamping.Hessian(f, fdot);
    const Vec9<Real> rate = Vectorize(fdot);
    EXPECT_NEAR(rate.dot(h * rate) / 2, kDamping.Psi(f, fdot),
                1e-12 * std::max(kDamping.Psi(f, fdot), Real(1)));
    const auto eigenvalues =
        Eigen::SelfAdjointEigenSolver<Mat9<Real>>(h, Eigen::EigenvaluesOnly)
            .eigenvalues();
    EXPECT_GE(eigenvalues.minCoeff(),
              -1e-12 * eigenvalues.cwiseAbs().maxCoeff());
  }
}

// Fdot = W F with W skew is a rigid rotation rate: no strain rate, so no
// stress and no dissipation. The control is Fdot = S F with S symmetric, a
// stretch rate, on the near-rest draws.
TEST(GreenDamping, RigidMotionIsNotDamped) {
  std::mt19937 rng(17);
  std::normal_distribution<Real> normal;
  Real worstRigid = 0;
  Real smallestStretch = std::numeric_limits<Real>::infinity();
  for (int n = 0; n < std::ssize(Samples()); ++n) {
    const Mat3<Real> &f = Samples()[n].first;
    Mat3<Real> a;
    for (int k = 0; k < 9; ++k)
      a(k) = normal(rng);
    const Real scale = 2 * kDamping.mu * f.squaredNorm();

    const Mat3<Real> spin = (a - a.transpose()) * f;
    const Real rigid = kDamping.Pk1(f, spin).norm() / (scale * spin.norm());
    worstRigid = std::max(worstRigid, rigid);
    const Real psiBound = 1e-14 * f.squaredNorm() * spin.norm();
    EXPECT_LE(kDamping.Psi(f, spin), kDamping.mu * psiBound * psiBound);

    const Mat3<Real> stretch = (a + a.transpose()) * f;
    if (n % 2 == 0)
      smallestStretch =
          std::min(smallestStretch,
                   kDamping.Pk1(f, stretch).norm() / (scale * stretch.norm()));
  }
  std::cout << "[ measured ] rigid |P| / (2 mu |F|^2 |Fdot|) at most "
            << worstRigid << "; a stretch rate gives at least "
            << smallestStretch << "\n";
  EXPECT_LE(worstRigid, 1e-14);
  EXPECT_GT(smallestStretch, 1e-2);
}

// The samples' Fdot as a step from f0 = f - h Fdot to f.
constexpr Real kStep = 0.05;

TEST(GreenDamping, StepPk1IsTheGradientOfStepPsi) {
  Real worst = 0;
  for (const auto &[f, fdot] : Samples()) {
    const Mat3<Real> f0 = f - kStep * fdot;
    Mat3<Real> fd;
    for (int k = 0; k < 9; ++k) {
      Mat3<Real> plus = f;
      Mat3<Real> minus = f;
      plus(k) += kFdStep;
      minus(k) -= kFdStep;
      fd(k) = (kDamping.StepPsi(plus, f0, kStep) -
               kDamping.StepPsi(minus, f0, kStep)) /
              (2 * kFdStep);
    }
    worst = std::max(worst, RelErr(kDamping.StepPk1(f, f0, kStep), fd));
  }
  EXPECT_LE(worst, 1e-6);
}

// As the step shrinks at a fixed rate, StepPsi / h and StepPk1 tend to Psi
// and Pk1, with an error of order h.
TEST(GreenDamping, StepFormTendsToTheRateFormAsTheStepShrinks) {
  for (const Real h : {1e-3, 1e-6}) {
    Real worstPsi = 0;
    Real worstPk1 = 0;
    for (const auto &[f, fdot] : Samples()) {
      const Mat3<Real> f0 = f - h * fdot;
      worstPsi = std::max(worstPsi,
                          std::abs(kDamping.StepPsi(f, f0, h) / h -
                                   kDamping.Psi(f, fdot)) /
                              std::max(kDamping.Psi(f, fdot), Real(1)));
      worstPk1 = std::max(worstPk1, (kDamping.StepPk1(f, f0, h) -
                                     kDamping.Pk1(f, fdot))
                                            .norm() /
                                        std::max(kDamping.Pk1(f, fdot).norm(),
                                                 Real(1)));
    }
    EXPECT_LE(worstPsi, 100 * h) << "h " << h;
    EXPECT_LE(worstPk1, 100 * h) << "h " << h;
  }
}

// A step that turns F by a finite rotation leaves E unchanged, so it
// dissipates nothing, where the rate form at the chord rate (f - f0) / h
// does.
TEST(GreenDamping, StepThroughAFiniteRotationIsNotDamped) {
  const Mat3<Real> turn =
      Eigen::AngleAxis<Real>(0.6, Vec3<Real>(1, 2, 3).normalized())
          .toRotationMatrix();
  Real worst = 0;
  Real chord = std::numeric_limits<Real>::infinity();
  for (const auto &[f0, unused] : Samples()) {
    const Mat3<Real> f = turn * f0;
    const Real scale = kDamping.mu * f0.squaredNorm() * f0.squaredNorm();
    worst = std::max(worst, kDamping.StepPsi(f, f0, kStep) / scale);
    worst = std::max(worst, kDamping.StepPk1(f, f0, kStep).norm() / scale);
    chord = std::min(chord,
                     kStep * kDamping.Psi(f, (f - f0) / kStep) / scale);
  }
  std::cout << "[ measured ] a 0.6 rad turn: step form at most " << worst
            << " of mu |F|^4, chord rate form at least " << chord << "\n";
  EXPECT_LE(worst, 1e-13);
  EXPECT_GT(chord, 1e-3);
}

} // namespace
