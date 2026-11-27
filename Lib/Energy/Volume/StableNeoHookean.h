#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Stable Neo-Hookean (Smith et al. 2018) in the form of Dynamic Deformables,
// "Stable Neo-Hookean": Psi = mu/2 (|F|^2 - 3) - mu (J - 1) + l/2 (J - 1)^2,
// J = det F, with no log regularizer. l = lame.lambda + lame.mu, the
// reparameterization that matches linear elasticity at rest.
struct StableNeoHookean {
  Lame lame;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<StableNeoHookean>);

} // namespace sim
