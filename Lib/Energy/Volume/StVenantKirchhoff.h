#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// St. Venant-Kirchhoff (Dynamic Deformables, "St. Venant-Kirchhoff"):
// Psi = mu |E|^2 + lambda/2 tr(E)^2 with Green strain E = (F^T F - I) / 2.
// It is a polynomial in F, so it is defined for inverted F, but its stress
// does not resist inversion.
struct StVenantKirchhoff {
  Lame lame;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<StVenantKirchhoff>);

} // namespace sim
