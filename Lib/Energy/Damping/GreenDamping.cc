#include "GreenDamping.h"

namespace sim {

namespace {

auto GreenStrainRate(const Mat3<Real> &f, const Mat3<Real> &fdot)
    -> Mat3<Real> {
  return (fdot.transpose() * f + f.transpose() * fdot) / 2;
}

} // namespace

auto GreenDamping::Psi(const Mat3<Real> &f, const Mat3<Real> &fdot) const
    -> Real {
  return mu * GreenStrainRate(f, fdot).squaredNorm();
}

// dPsi = 2 mu Edot : dEdot = 2 mu Edot : F^T dFdot, since Edot is symmetric,
// so P = 2 mu F Edot.
auto GreenDamping::Pk1(const Mat3<Real> &f, const Mat3<Real> &fdot) const
    -> Mat3<Real> {
  return 2 * mu * f * GreenStrainRate(f, fdot);
}

// P is linear in Fdot, so column 3 b + a is vec(P(F, e_a e_b^T)), whatever
// Fdot is. Psi is a quadratic form in Fdot with this Hessian, and Psi >= 0,
// so the Hessian is PSD without clamping.
auto GreenDamping::Hessian(const Mat3<Real> &f,
                           const Mat3<Real> & /*fdot*/) const -> Mat9<Real> {
  Mat9<Real> h;
  for (int b = 0; b < 3; ++b) {
    for (int a = 0; a < 3; ++a) {
      Mat3<Real> unit = Mat3<Real>::Zero();
      unit(a, b) = 1;
      h.col(3 * b + a) = Vectorize(Pk1(f, unit));
    }
  }
  return h;
}

auto GreenDamping::StepPsi(const Mat3<Real> &f, const Mat3<Real> &f0,
                           Real h) const -> Real {
  return h * Psi((f0 + f) / 2, (f - f0) / h);
}

// dE(f) = sym(f^T df), so dStepPsi = 2 mu Edot : sym(f^T df) with Edot the
// mean rate, and dStepPsi/df = 2 mu f Edot.
auto GreenDamping::StepPk1(const Mat3<Real> &f, const Mat3<Real> &f0,
                           Real h) const -> Mat3<Real> {
  return 2 * mu * f * GreenStrainRate((f0 + f) / 2, (f - f0) / h);
}

} // namespace sim
