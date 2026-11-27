#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Symmetric Dirichlet (Smith et al. 2019): Psi = mu/4 (|F|^2 + |F^-1|^2 - 6),
// or mu/4 sum (s_i^2 + s_i^-2 - 2) in singular values. The 1/4 makes its rest
// Hessian 2 mu on every non-twist mode, the same shear stiffness as linear
// elasticity. lame.lambda is unused.
//
// Domain: det F != 0. Psi grows like 1 / s^2 as any singular value goes to
// 0, so it is singular there and undefined at det F = 0. It is defined for
// inverted F (s_3 < 0), where it equals Psi of the reflected F.
struct SymmetricDirichlet {
  Lame lame;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<SymmetricDirichlet>);

} // namespace sim
