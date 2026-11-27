#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Anisotropic St. Venant-Kirchhoff (Kim et al. 2019): StVK restricted to the
// Green strain along a unit fiber a, E_a = a^T E a = (I5 - 1) / 2 with
// I5 = |F a|^2, and Psi = mu/2 E_a^2 = mu/8 (I5 - 1)^2. The scale gives fiber
// stretch e the rest stiffness Psi ~ mu/2 e^2, the same as AnisotropicArap.
// lame.lambda is unused.
//
// It resists only stretch along the fiber, so it is meant to be added to an
// isotropic energy. Like StVK it is a polynomial, defined for every F, and it
// does not resist inversion. fiber is uniform over the body and must be unit
// length.
struct AnisotropicStVenantKirchhoff {
  Lame lame;
  Vec3<Real> fiber = Vec3<Real>::UnitX();

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<AnisotropicStVenantKirchhoff>);

} // namespace sim
