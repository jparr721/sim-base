#pragma once

#include <LibMath.h>

class MassSpring {
public:
  // Spring stiffness
  Real k;

  explicit MassSpring(Real k) : k(k) {}
  [[nodiscard]] auto Psi(const Vec6<Real> &x, Real restLength) const -> Real;
  [[nodiscard]] auto Gradient(const Vec6<Real> &x, Real restLength) const
      -> Vec6<Real>;
};
