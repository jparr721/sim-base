#include "MassSpring.h"

auto MassSpring::Psi(const Vec6<Real> &x, Real restLength) const -> Real {
  const Vec3<Real> p0 = x.segment<3>(0);
  const Vec3<Real> p1 = x.segment<3>(3);
  const Real diff = (p1 - p0).norm() - restLength;
  return k / 2 * diff * diff;
}

auto MassSpring::Gradient(const Vec6<Real> &x, Real restLength) const
    -> Vec6<Real> {
  const Vec3<Real> p0 = x.segment<3>(0);
  const Vec3<Real> p1 = x.segment<3>(3);
  const Real diffNorm = (p1 - p0).norm();

  Vec6<Real> mult = Vec6<Real>::Zero();
  mult.segment<3>(0) = (p0 - p1);
  mult.segment<3>(3) = (p1 - p0);

  return k * mult * (diffNorm - restLength) / diffNorm;
}
