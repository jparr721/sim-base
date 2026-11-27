#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// As-rigid-as-possible (Smith et al. 2019; Dynamic Deformables,
// "ARAP"): Psi = mu |F - R|^2 with R = U V^T from the rotation-variant SVD,
// so Psi = mu sum (s_i - 1)^2. lame.lambda is unused. Psi is continuous for
// every F, but R, and so P, jumps where s_i + s_j = 0 for a pair of singular
// values: the Hessian holds 1 / (s_i + s_j) and is singular there.
struct Arap {
  Lame lame;

  [[nodiscard]] auto Psi(const Mat3<Real> &f) const -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f) const -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f) const -> Mat9<Real>;
  [[nodiscard]] auto ClampedHessian(const Mat3<Real> &f, const Svd &svd) const
      -> Mat9<Real>;
};

static_assert(Energy<Arap>);

} // namespace sim
