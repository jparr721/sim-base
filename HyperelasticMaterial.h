#pragma once

#include "Math.h"
#include "Settings.h"
#include <cfloat>

INLINE auto ComputeMu(Real E, Real nu) -> Real { return E / (2 * (1 + nu)); }

INLINE auto ComputeLambda(Real E, Real nu) -> Real {
  return E * nu / ((1 + nu) * (1 - 2 * nu));
}

INLINE auto CauchyInvariant(const Mat3<Real> &F) -> Real {
  return F.squaredNorm();
}

class HyperelasticMaterial {
public:
  HyperelasticMaterial(Real E, Real nu)
      : mu(ComputeMu(E, nu)), lambda(ComputeLambda(E, nu)){};
  virtual ~HyperelasticMaterial() = default;

  [[nodiscard]] virtual auto Name() -> std::string = 0;
  [[nodiscard]] virtual auto Psi(const Mat3<Real> &F) const -> Real = 0;
  [[nodiscard]] virtual auto Pk1(const Mat3<Real> &F) const -> Mat<Real> = 0;
  [[nodiscard]] virtual auto Hessian(const Mat3<Real> &F) const
      -> Mat<Real> = 0;

  virtual auto FiniteDifferenceTestPk1(const Mat3<Real> &F) -> bool {
    std::cout << "===========" << std::endl;
    std::cout << "PK1 Test for " << Name() << std::endl;
    std::cout << "===========" << std::endl;
    auto psi0 = Psi(F);
    auto pk10 = Pk1(F);

    auto eps = 1e-4;
    int e = 0;
    auto minSeen = DBL_MAX;
    while (eps > 1e-8) {
      Mat3<Real> finiteDiff;

      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          Mat3<Real> Fnew = F;

          // Perturb the input slightly
          Fnew(ii, jj) += eps;

          // Compute the new energy
          auto psiP = Psi(Fnew);

          // Store the finite difference
          finiteDiff(ii, jj) = (psiP - psi0) / eps;
        }
      }

      // Compute the error
      Mat3<Real> error = pk10 - finiteDiff;
      Real diffNorm = fabs(error.norm() / pk10.norm()) / 9.0;

      std::cout << "eps: " << eps << " diffNorm: " << diffNorm << std::endl;

      if (diffNorm < minSeen) {
        minSeen = diffNorm;
      }

      if (e == 4 && minSeen > 1e-6) {
        std::cout << "Pk1 finite difference test failed" << std::endl;
        std::cout << "pk10: " << pk10 << std::endl;
        std::cout << "finiteDiff: " << finiteDiff << std::endl;
        std::cout << "error: " << error << std::endl;
        return false;
      } else {
        eps *= 0.1;
      }
      ++e;
    }

    if (minSeen < 1e-6) {
      std::cout << "Pk1 finite difference test passed" << std::endl;
      return true;
    } else {
      std::cout << "Pk1 finite difference test failed" << std::endl;
      return false;
    }
  }

  virtual auto FiniteDifferenceTestHessian(const Mat3<Real> &F) -> bool {
    return true;
  }

protected:
  Real mu;
  Real lambda;
};
