#include <Energy/Volume/STVK.h>
auto STVK::Psi(const Mat3<Real> &F) const -> Real {
  Mat3<Real> E = 0.5 * (F.transpose() * F - Mat3<Real>::Identity());
  auto trE = E.trace();
  return mu * E.squaredNorm() + 0.5 * lambda * (trE * trE);
}
auto STVK::Pk1(const Mat3<Real> &F) const -> Mat<Real> {
  // E-Based Pk1
  Mat3<Real> E = 0.5 * (F.transpose() * F - Mat3<Real>::Identity());
  return F * (2.0 * mu * E + lambda * E.trace() * Mat3<Real>::Identity());

  // F-based Pk1, this one doesn't work though, not sure why.
  //        Mat3<Real> E = 0.5 * (F.transpose() * F - Mat3<Real>::Identity());
  //        auto trE = E.trace();
  //        return mu * F * E + lambda * trE * F;
}
auto STVK::Hessian(const Mat3<Real> &F) const -> Mat<Real> {
  return Mat<Real>();
}
