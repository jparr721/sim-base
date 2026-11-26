#include "MassSpring.h"

#include <algorithm>

namespace {

auto EdgeVector(const VecN<6> &x) -> Vec3<Real> {
  return x.segment<3>(3) - x.segment<3>(0);
}

// The 6x6 Hessian of an energy of e = x1 - x0 whose Hessian in e is h.
auto Stack(const Mat3<Real> &h) -> MatN<6> {
  MatN<6> out;
  out << h, -h, -h, h;
  return out;
}

} // namespace

auto MassSpring::Psi(const VecN<6> &x) const -> Real {
  const Real l = EdgeVector(x).norm();
  return k * (l * l * l / 3 - restLength * l * l / 2);
}

auto MassSpring::Gradient(const VecN<6> &x) const -> VecN<6> {
  const Vec3<Real> e = EdgeVector(x);
  const Real l = e.norm();

  VecN<6> mult;
  mult << -e, e;

  // Analytically k (l - d) mult. Evaluated as |e| times the quadratic spring's
  // gradient so the strand forces keep their floating-point results.
  const VecN<6> quadraticGradient = k * mult * (l - restLength) / l;
  return l * quadraticGradient;
}

// In e: k (l - d) I + k l t t^T, with t = e / l.
auto MassSpring::Hessian(const VecN<6> &x) const -> MatN<6> {
  const Vec3<Real> e = EdgeVector(x);
  const Real l = e.norm();
  return Stack(k * (l - restLength) * Mat3<Real>::Identity() +
               k / l * e * e.transpose());
}

// The Hessian in e has eigenvalue k (2l - d) along t and the tension over
// length k (l - d) twice on the plane perpendicular to t.
auto MassSpring::ClampedHessian(const VecN<6> &x) const -> MatN<6> {
  const Vec3<Real> e = EdgeVector(x);
  const Real l = e.norm();
  const Vec3<Real> t = e / l;
  const Mat3<Real> axial = t * t.transpose();
  const Real axialEigenvalue = std::max<Real>(k * (2 * l - restLength), 0);
  const Real transverseEigenvalue = std::max<Real>(k * (l - restLength), 0);
  return Stack(axialEigenvalue * axial +
               transverseEigenvalue * (Mat3<Real>::Identity() - axial));
}
