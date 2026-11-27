#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// A Fung-type exponential fiber energy: Psi = mu / (2 k) (exp(k E_a^2) - 1)
// with E_a = (I5 - 1) / 2 the Green strain along a unit fiber a and
// k = stiffening > 0. Fung's exponential form c/2 (e^Q - 1) with Q = k E_a^2,
// restricted to the fiber. It stiffens exponentially with fiber strain, the
// way collagen-reinforced tissue does, and as k -> 0 it becomes
// AnisotropicStVenantKirchhoff. The form is chosen here and derived by hand
// on the Psi(I5) eigensystem of Kim et al. 2019. lame.lambda is unused.
//
// It is defined for every F and does not resist inversion. It resists only
// stretch along the fiber, so it is meant to be added to an isotropic
// energy. fiber is uniform over the body and must be unit length.
struct AnisotropicFung {
  Lame lame;
  Vec3<Real> fiber = Vec3<Real>::UnitX();
  Real stiffening = 1;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<AnisotropicFung>);

} // namespace sim
