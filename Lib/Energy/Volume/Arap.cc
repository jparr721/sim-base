#include "Arap.h"

namespace sim {

namespace {

// dPsi/ds_i = 2 mu (s_i - 1), so the scaling block is 2 mu I, the flip
// eigenvalues (psi_i - psi_j) / (s_i - s_j) are 2 mu, and the twist
// eigenvalues (psi_i + psi_j) / (s_i + s_j) are 2 mu (1 - 2 / (s_i + s_j))
// (Smith et al. 2019). Only the twists can be negative.
auto ArapHessian(const Lame &lame, const Svd &svd, bool clamped)
    -> Mat9<Real> {
  const Real twoMu = 2 * lame.mu;
  const Vec3<Real> &s = svd.sigma;
  const Vec3<Real> twist(twoMu * (1 - 2 / (s(1) + s(2))),
                         twoMu * (1 - 2 / (s(0) + s(2))),
                         twoMu * (1 - 2 / (s(0) + s(1))));
  return IsotropicHessian(svd, twoMu * Mat3<Real>::Identity(), twist,
                          Vec3<Real>::Constant(twoMu), clamped);
}

} // namespace

auto Arap::Psi(const Mat3<Real> &f) const -> Real {
  return lame.mu * (RotationVariantSvd(f).sigma.array() - 1).square().sum();
}

// P = 2 mu (F - R).
auto Arap::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  const Svd svd = RotationVariantSvd(f);
  return 2 * lame.mu * (f - svd.u * svd.v.transpose());
}

auto Arap::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  return ArapHessian(lame, RotationVariantSvd(f), false);
}

auto Arap::ClampedHessian(const Mat3<Real> & /*f*/, const Svd &svd) const
    -> Mat9<Real> {
  return ArapHessian(lame, svd, true);
}

} // namespace sim
