#include "AnisotropicArap.h"

namespace sim {

namespace {

// S(I4) with I4 = a^T V Sigma V^T a.
auto FiberSign(const Svd &svd, const Vec3<Real> &a) -> Real {
  const Vec3<Real> va = svd.v.transpose() * a;
  return va.dot(svd.sigma.cwiseProduct(va)) >= 0 ? 1 : -1;
}

// Psi as a function of I5 for a fixed sign s:
// Psi' = mu/2 (1 - s / sqrt(I5)) and Psi'' = mu s / (4 I5^(3/2)).
struct FiberTerm {
  Real dpsi;
  Real d2psi;
};

auto FiberTermOf(const Lame &lame, const Mat3<Real> &f, const Vec3<Real> &a,
                 Real sign) -> FiberTerm {
  const Real root = (f * a).norm();
  return {.dpsi = lame.mu / 2 * (1 - sign / root),
          .d2psi = lame.mu * sign / (4 * root * root * root)};
}

} // namespace

auto AnisotropicArap::Psi(const Mat3<Real> &f) const -> Real {
  const Real stretch =
      (f * fiber).norm() - FiberSign(RotationVariantSvd(f), fiber);
  return lame.mu / 2 * stretch * stretch;
}

auto AnisotropicArap::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  const Real sign = FiberSign(RotationVariantSvd(f), fiber);
  return FiberPk1(f, fiber, FiberTermOf(lame, f, fiber, sign).dpsi);
}

// Along F a a^T the eigenvalue 2 Psi' + 4 I5 Psi'' is mu for either sign.
// Across it, 2 Psi' = mu (1 - s / sqrt(I5)) is negative when the fiber is
// compressed and not inverted (Kim et al. 2019).
auto AnisotropicArap::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  const FiberTerm t =
      FiberTermOf(lame, f, fiber, FiberSign(RotationVariantSvd(f), fiber));
  return FiberHessian(f, fiber, t.dpsi, t.d2psi, false);
}

auto AnisotropicArap::ClampedHessian(const Mat3<Real> &f,
                                     const Svd &svd) const -> Mat9<Real> {
  const FiberTerm t = FiberTermOf(lame, f, fiber, FiberSign(svd, fiber));
  return FiberHessian(f, fiber, t.dpsi, t.d2psi, true);
}

} // namespace sim
