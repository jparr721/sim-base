#pragma once

#include <Energy/HyperelasticMaterial.h>

class SNH : public HyperelasticMaterial {
public:
  SNH(Real E, Real nu) : HyperelasticMaterial(E, nu) { lambda = lambda + mu; }
  [[nodiscard]] INLINE auto Name() -> std::string override { return "SNH"; }
  [[nodiscard]] auto Psi(const Mat3<Real> &F) const -> Real override;
  [[nodiscard]] auto Pk1(const Mat3<Real> &F) const -> Mat<Real> override;
  [[nodiscard]] auto Hessian(const Mat3<Real> &F) const -> Mat<Real> override;

  [[nodiscard]] static auto PJPF(const Mat3<Real> &F) -> Mat3<Real>;

  //////////////////////////////////////////////////////////////////////////////
  // Eqn. 29 from Section 4.5 in "Stable Neo-Hookean Flesh Simulation",
  // with a scaling factor added
  //////////////////////////////////////////////////////////////////////////////
  static auto CrossProduct(const Mat3<Real> &F, int i) -> Mat3<Real>;

  //////////////////////////////////////////////////////////////////////////////
  // cross product matrix
  //////////////////////////////////////////////////////////////////////////////
  static auto CrossProduct(const Vec3<Real> &x) -> Mat3<Real>;

  // The third tensor invariant
  static auto Invariant3(const Mat3<Real> &F) -> Real;
};