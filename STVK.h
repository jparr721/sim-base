#pragma once

#include "HyperelasticMaterial.h"

class STVK : public HyperelasticMaterial {
public:
  STVK(Real E, Real nu) : HyperelasticMaterial(E, nu) {}
  [[nodiscard]] INLINE auto Name() -> std::string override { return "STVK"; }
  [[nodiscard]] INLINE auto Psi(const Mat3<Real> &F) const -> Real override {
    Mat3<Real> E = 0.5 * (F.transpose() * F - Mat3<Real>::Identity());
    auto trE = E.trace();
    return mu * E.squaredNorm() + 0.5 * lambda * (trE * trE);
  }

  [[nodiscard]] INLINE auto Pk1(const Mat3<Real> &F) const
      -> Mat<Real> override {
    // E-Based Pk1
    Mat3<Real> E = 0.5 * (F.transpose() * F - Mat3<Real>::Identity());
    return F * (2.0 * mu * E + lambda * E.trace() * Mat3<Real>::Identity());

    // F-based Pk1, this one doesn't work though, not sure why.
    //    Mat3<Real> E = 0.5 * (F.transpose() * F - Mat3<Real>::Identity());
    //    auto trE = E.trace();
    //    return mu * F * E + lambda * trE * F;
  }

  [[nodiscard]] INLINE auto Hessian(const Mat3<Real> &F) const
      -> Mat<Real> override {
    return Mat<Real>();
  }

private:
};