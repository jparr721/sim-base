#pragma once

#include "HyperelasticMaterial.h"

class SNH : public HyperelasticMaterial {
public:
  SNH(Real E, Real nu) : HyperelasticMaterial(E, nu) {}
  [[nodiscard]] INLINE auto Name() -> std::string override { return "SNH"; }
  [[nodiscard]] INLINE auto Psi(const Mat3<Real> &F) const -> Real override {
    Real Ic = CauchyInvariant(F);
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

  auto Hessian(const Mat3<Real> &F) const -> Mat<Real> override {
    return Mat<Real>();
  }

  [[nodiscard]] INLINE auto PJPF(const Mat3<Real> &F) const -> Mat3<Real> {
    //    Mat3<Real> pJpF = Mat3<Real>::Zero();
    //    const Real f00 = F(0, 0);
    //    const Real f01 = F(0, 1);
    //    const Real f02 = F(0, 2);
    //    const Real f10 = F(1, 0);
    //    const Real f11 = F(1, 1);
    //    const Real f12 = F(1, 2);
    //    const Real f20 = F(2, 0);
    //    const Real f21 = F(2, 1);
    //    const Real f22 = F(2, 2);
    //
    //    pJpF(0, 0) = f11 * f22 - f12 * f21;
    //    pJpF(0, 1) = f10 * f22 - f12 * f20;
    //    pJpF(0, 2) = f10 * f21 - f11 * f20;
    //    pJpF(1, 0) = f02 * f21 - f01 * f22;
    //    pJpF(1, 1) = f00 * f22 - f02 * f20;
    //    pJpF(1, 2) = f01 * f20 - f00 * f21;
    //    pJpF(2, 0) = f01 * f12 - f02 * f11;
    //    pJpF(2, 1) = f02 * f10 - f00 * f12;
    //    pJpF(2, 2) = f00 * f11 - f01 * f10;
    //    return pJpF;

    //    Mat3<Real> pJpF;
    //    pJpF.col(0) = F.col(1).cross(F.col(2));
    //    pJpF.col(1) = F.col(2).cross(F.col(0));
    //    pJpF.col(2) = F.col(0).cross(F.col(1));
    //    return pJpF;

    const Real f11 = F(0, 0);
    const Real f12 = F(0, 1);
    const Real f13 = F(0, 2);
    const Real f21 = F(1, 0);
    const Real f22 = F(1, 1);
    const Real f23 = F(1, 2);
    const Real f31 = F(2, 0);
    const Real f32 = F(2, 1);
    const Real f33 = F(2, 2);

    Mat3<Real> pJpF;
    pJpF(0, 0) = f22 * f33 - f23 * f32;
    pJpF(0, 1) = -f21 * f33 + f23 * f31;
    pJpF(0, 2) = f21 * f32 - f22 * f31;
    pJpF(1, 0) = -f12 * f33 + f13 * f32;
    pJpF(1, 1) = f11 * f33 - f13 * f31;
    pJpF(1, 2) = -f11 * f32 + f12 * f31;
    pJpF(2, 0) = f12 * f23 - f13 * f22;
    pJpF(2, 1) = -f11 * f23 + f13 * f21;
    pJpF(2, 2) = f11 * f22 - f12 * f21;
    return pJpF;
  }
};