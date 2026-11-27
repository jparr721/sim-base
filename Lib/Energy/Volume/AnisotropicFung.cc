#include "AnisotropicFung.h"

namespace sim {

namespace {

auto FiberStrain(const Mat3<Real> &f, const Vec3<Real> &a) -> Real {
  return ((f * a).squaredNorm() - 1) / 2;
}

// With x = exp(k E^2) and dE/dI5 = 1/2: Psi'(I5) = mu E x / 2 and
// Psi''(I5) = mu x (1 + 2 k E^2) / 4.
auto FungHessian(const AnisotropicFung &e, const Mat3<Real> &f, bool clamped)
    -> Mat9<Real> {
  const Real strain = FiberStrain(f, e.fiber);
  const Real x = std::exp(e.stiffening * strain * strain);
  return FiberHessian(
      f, e.fiber, e.lame.mu * strain * x / 2,
      e.lame.mu * x * (1 + 2 * e.stiffening * strain * strain) / 4, clamped);
}

} // namespace

auto AnisotropicFung::Psi(const Mat3<Real> &f) const -> Real {
  const Real strain = FiberStrain(f, fiber);
  return lame.mu / (2 * stiffening) * std::expm1(stiffening * strain * strain);
}

auto AnisotropicFung::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  const Real strain = FiberStrain(f, fiber);
  return FiberPk1(f, fiber,
                  lame.mu * strain *
                      std::exp(stiffening * strain * strain) / 2);
}

auto AnisotropicFung::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  return FungHessian(*this, f, false);
}

auto AnisotropicFung::ClampedHessian(const Mat3<Real> &f,
                                     const Svd & /*svd*/) const -> Mat9<Real> {
  return FungHessian(*this, f, true);
}

} // namespace sim
