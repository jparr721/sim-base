#pragma once

#include <LibMath.h>

class MassSpring {
public:
  // Spring stiffness
  Real k;

  // Spring rest length
  Real d;

  Real kover2;

  MassSpring(Real k, Real d) : k(k), d(d), kover2(k / 2) {}
  [[nodiscard]] auto Psi(const Vec6<Real> &x) const -> Real;
  [[nodiscard]] auto Gradient(const Vec6<Real> &x) const -> Vec6<Real>;
};
