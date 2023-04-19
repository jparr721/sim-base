#pragma once

#include <Energy/Volume/HyperelasticMaterial.h>

class STVK : public HyperelasticMaterial {
public:
  STVK(Real E, Real nu) : HyperelasticMaterial(E, nu) {}
  [[nodiscard]] INLINE auto Name() -> std::string override { return "STVK"; }
  [[nodiscard]] auto Psi(const Mat3<Real> &F) const -> Real override;
  [[nodiscard]] auto Pk1(const Mat3<Real> &F) const
      -> Mat<Real> override;
  [[nodiscard]] auto Hessian(const Mat3<Real> &F) const
      -> Mat<Real> override;

private:
};