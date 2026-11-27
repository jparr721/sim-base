#pragma once

#include <Energy/Volume/Energy.h>

namespace sim {

// Damping on the rate of Green strain (Kim & Eberle 2022, Dynamic
// Deformables, damping chapter): Psi = mu |Edot|^2 with
// Edot = (Fdot^T F + F^T Fdot) / 2, the time derivative of E = (F^T F - I) / 2.
// Its second Piola-Kirchhoff stress is the viscous stress 2 mu Edot. Edot is
// zero when Fdot = W F with W skew, so rigid motion is never damped. mu is a
// viscosity, in stress times seconds.
struct GreenDamping {
  Real mu;

  [[nodiscard]] auto Psi(const Mat3<Real> &f, const Mat3<Real> &fdot) const
      -> Real;
  [[nodiscard]] auto Pk1(const Mat3<Real> &f, const Mat3<Real> &fdot) const
      -> Mat3<Real>;
  [[nodiscard]] auto Hessian(const Mat3<Real> &f, const Mat3<Real> &fdot) const
      -> Mat9<Real>;
};

static_assert(DampingEnergy<GreenDamping>);

} // namespace sim
