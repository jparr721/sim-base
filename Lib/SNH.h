#pragma once

#include "HyperelasticMaterial.h"

class SNH : public HyperelasticMaterial {
public:
  SNH(Real E, Real nu) : HyperelasticMaterial(E, nu) {}
  [[nodiscard]] INLINE auto Name() -> std::string override { return "SNH"; }
  [[nodiscard]] INLINE auto Psi(const Mat3<Real> &F) const -> Real override {
    // Equation [13] in [Smith et al. 2018]
    Real Ic = CauchyInvariant(F);

    // This term is from equation 14, we combine them here and ignore the
    // regularizing term from the equation since it doesn't really matter.
    const Real alpha = 1.0 + mu / lambda;
    Real Jminus1 = F.determinant() - alpha;
    return 0.5 * (mu * (Ic - 3.0) + lambda * Jminus1 * Jminus1);
  }

  [[nodiscard]] INLINE auto Pk1(const Mat3<Real> &F) const
      -> Mat<Real> override {
    Real J = F.determinant();
    const Mat3<Real> pJpF = PJPF(F);

    return mu * F - mu * pJpF + lambda * (J - 1) * pJpF;
  }

  [[nodiscard]] auto Hessian(const Mat3<Real> &F) const -> Mat<Real> override {
    return Mat<Real>();
  }

  [[nodiscard]] INLINE auto PJPF(const Mat3<Real> &F) const -> Mat3<Real> {
    Mat3<Real> pJpF;
    pJpF.col(0) = F.col(1).cross(F.col(2));
    pJpF.col(1) = F.col(2).cross(F.col(0));
    pJpF.col(2) = F.col(0).cross(F.col(1));
    return pJpF;
  }
};