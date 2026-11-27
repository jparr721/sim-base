#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// The classic compressible Neo-Hookean energy (Bonet and Wood; Smith et al.
// 2018, Eq. 2): Psi = mu/2 (|F|^2 - 3) - mu log J + lambda/2 (log J)^2 with
// J = det F.
//
// Inversion guard: log J is singular at J = 0 and undefined below it. For
// J < kGuardJ, log J is replaced by its second-order Taylor expansion about
// kGuardJ, L(J) = log g + (J - g)/g - (J - g)^2 / (2 g^2). L matches log in
// value, slope and curvature at g, so Psi is C^2. L keeps increasing in J
// for every J < g, so Psi is finite for every F, including J <= 0, and keeps
// growing as J shrinks through zero into inversion. For J >= kGuardJ the
// energy is exactly the classic one.
struct NeoHookeanBonetWood {
  Lame lame;

  static constexpr Real kGuardJ = 0.1;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<NeoHookeanBonetWood>);

} // namespace sim
