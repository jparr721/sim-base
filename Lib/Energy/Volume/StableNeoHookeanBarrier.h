#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Stable Neo-Hookean plus an inversion barrier on J = det F:
// Psi = Psi_SNH + b(J), with Psi_SNH as in StableNeoHookean and
//   b(J) = -l (J - g)^2 log(J / g)  for 0 < J < g = kBarrierJ,
//   b(J) = 0                         for J >= g.
// l = lame.lambda + lame.mu is SNH's volume stiffness. b and its first two
// derivatives vanish at g, so Psi is C^2 and equals SNH wherever J >= g, and
// b grows without bound as J -> 0+. The barrier shape is the clamped log
// barrier of Li et al. 2020 (IPC) applied to J, derived here by hand.
//
// Domain: J > 0. For J <= 0, Psi is +infinity, so a line search rejects a
// step that inverts an element. Pk1, Hessian and ClampedHessian there drop
// the barrier and return SNH's terms, so an explicit step that inverts an
// element keeps SNH's inversion handling instead of producing NaN.
struct StableNeoHookeanBarrier {
  Lame lame;

  static constexpr Real kBarrierJ = 0.25;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<StableNeoHookeanBarrier>);

} // namespace sim
