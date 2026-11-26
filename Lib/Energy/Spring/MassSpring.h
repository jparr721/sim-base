#pragma once

#include <Energy/ElementEnergy.h>

// A cubic spring on one edge x = (x0, x1): Psi = k (l^3/3 - d l^2/2) with
// l = |x1 - x0| and d the rest length. Its gradient is |e| times the gradient
// of the quadratic spring k/2 (l - d)^2.
struct MassSpring {
  Real k;
  Real restLength;

  [[nodiscard]] auto Psi(const VecN<6> &x) const -> Real;
  [[nodiscard]] auto Gradient(const VecN<6> &x) const -> VecN<6>;
  [[nodiscard]] auto Hessian(const VecN<6> &x) const -> MatN<6>;
  [[nodiscard]] auto ClampedHessian(const VecN<6> &x) const -> MatN<6>;
};

static_assert(ElementEnergy<MassSpring, 6>);
