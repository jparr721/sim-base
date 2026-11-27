#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Anisotropic ARAP (Kim et al. 2019): Psi = mu/2 (sqrt(I5) - S(I4))^2 for a
// unit fiber a, with I5 = |F a|^2, I4 = a^T S a for the stretch S = V Sigma
// V^T of the rotation-variant SVD, and S(I4) = 1 if I4 >= 0, else -1. The
// sign makes it inversion-aware: when the fiber is inverted (I4 < 0) its
// rest length is -1, so the energy pushes it back through zero rather than
// settling inverted. lame.lambda is unused.
//
// Domain: Psi jumps where I4 crosses 0 and is not differentiable at I5 = 0
// (F a = 0), so it is smooth only where I4 != 0. It resists only stretch along
// the fiber, so it is meant to be added to an isotropic energy. fiber is
// uniform over the body and must be unit length.
struct AnisotropicArap {
  Lame lame;
  Vec3<Real> fiber = Vec3<Real>::UnitX();

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<AnisotropicArap>);

} // namespace sim
