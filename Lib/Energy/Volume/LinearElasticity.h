#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Small-strain linear elasticity (Dynamic Deformables, "Linear Elasticity"):
// Psi = mu |eps|^2 + lambda/2 tr(eps)^2 with eps = sym(F) - I. It is
// quadratic in F, so its Hessian is constant. It is not rotation invariant:
// a rotated element is strained, so it is only a reference for small
// deformations.
struct LinearElasticity {
  Lame lame;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<LinearElasticity>);

} // namespace sim
