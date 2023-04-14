#pragma once

#include "HyperelasticMaterial.h"

class SNH : public HyperelasticMaterial {
public:
  SNH(Real E, Real nu) : HyperelasticMaterial(E, nu) { lambda = lambda + mu; }
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
    const Real alpha = 1.0 + mu / lambda;
    Real Jminus1 = F.determinant() - alpha;
    const Mat3<Real> pJpF = PJPF(F);
    //    return mu * F + lambda * Jminus1 * pJpF;

    return mu * F - mu * pJpF + lambda * (J - 1) * pJpF;
  }

  [[nodiscard]] auto Hessian(const Mat3<Real> &F) const -> Mat<Real> override {
    const Vec9<Real> pjpf = Flatten(PJPF(F));

    const Real I3 = Invariant3(F);
    const Real alpha = 1.0 + mu / lambda;
    const Real scale = lambda * (I3 - alpha);

    const Mat3<Real> f0hat = CrossProduct(F, 0) * scale;
    const Mat3<Real> f1hat = CrossProduct(F, 1) * scale;
    const Mat3<Real> f2hat = CrossProduct(F, 2) * scale;

    // Build the fractal cross product
    Mat9<Real> hessJ = Mat9<Real>::Zero();

    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 3; ++i) {
        hessJ(i, j + 3) = -f2hat(i, j);
        hessJ(i + 3, j) = f2hat(i, j);

        hessJ(i, j + 6) = f1hat(i, j);
        hessJ(i + 6, j) = -f1hat(i, j);

        hessJ(i + 3, j + 6) = -f0hat(i, j);
        hessJ(i + 6, j + 3) = f0hat(i, j);
      }
    }

    return mu * Mat9<Real>::Identity() + lambda * pjpf * pjpf.transpose() +
           hessJ;
  }

  [[nodiscard]] INLINE auto PJPF(const Mat3<Real> &F) const -> Mat3<Real> {
    Mat3<Real> pJpF;
    pJpF.col(0) = F.col(1).cross(F.col(2));
    pJpF.col(1) = F.col(2).cross(F.col(0));
    pJpF.col(2) = F.col(0).cross(F.col(1));
    return pJpF;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Eqn. 29 from Section 4.5 in "Stable Neo-Hookean Flesh Simulation",
  // with a scaling factor added
  //////////////////////////////////////////////////////////////////////////////
  static auto CrossProduct(const Mat3<Real> &F, const int i) -> Mat3<Real> {
    return (Mat3<Real>() << 0, -F(2, i), F(1, i), F(2, i), 0, -F(0, i),
            -F(1, i), F(0, i), 0)
        .finished();
  }

  //////////////////////////////////////////////////////////////////////////////
  // cross product matrix
  //////////////////////////////////////////////////////////////////////////////
  static auto CrossProduct(const Vec3<Real> &x) -> Mat3<Real> {
    return (Mat3<Real>() << 0, -x[2], x[1], x[2], 0, -x[0], -x[1], x[0], 0)
        .finished();
  }

  // The third tensor invariant
  static auto Invariant3(const Mat3<Real> &F) -> Real {
    return F.determinant();
  }
};