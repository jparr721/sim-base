#include "AnisotropicStVenantKirchhoff.h"

namespace sim {

namespace {

auto FiberStrain(const Mat3<Real> &f, const Vec3<Real> &a) -> Real {
  return ((f * a).squaredNorm() - 1) / 2;
}

} // namespace

auto AnisotropicStVenantKirchhoff::Psi(const Mat3<Real> &f) const -> Real {
  const Real e = FiberStrain(f, fiber);
  return lame.mu / 2 * e * e;
}

// Psi'(I5) = mu E_a / 2.
auto AnisotropicStVenantKirchhoff::Pk1(const Mat3<Real> &f) const
    -> Mat3<Real> {
  return FiberPk1(f, fiber, lame.mu * FiberStrain(f, fiber) / 2);
}

// Psi''(I5) = mu / 4.
auto AnisotropicStVenantKirchhoff::Hessian(const Mat3<Real> &f) const
    -> Mat9<Real> {
  return FiberHessian(f, fiber, lame.mu * FiberStrain(f, fiber) / 2,
                      lame.mu / 4, false);
}

auto AnisotropicStVenantKirchhoff::ClampedHessian(const Mat3<Real> &f,
                                                  const Svd & /*svd*/) const
    -> Mat9<Real> {
  return FiberHessian(f, fiber, lame.mu * FiberStrain(f, fiber) / 2,
                      lame.mu / 4, true);
}

} // namespace sim
