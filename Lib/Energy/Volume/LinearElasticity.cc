#include "LinearElasticity.h"

namespace sim {

namespace {

auto SmallStrain(const Mat3<Real> &f) -> Mat3<Real> {
  return (f + f.transpose()) / 2 - Mat3<Real>::Identity();
}

// S vec(F) = vec(sym F): the projector onto symmetric matrices.
auto SymmetricProjector() -> Mat9<Real> {
  Mat9<Real> s = Mat9<Real>::Zero();
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      s(3 * j + i, 3 * j + i) += Real(0.5);
      s(3 * j + i, 3 * i + j) += Real(0.5);
    }
  }
  return s;
}

// vec(I) / sqrt3, the unit volumetric mode.
auto UnitVolumetric() -> Vec9<Real> {
  return Vectorize(Mat3<Real>::Identity()) / std::sqrt(Real(3));
}

} // namespace

auto LinearElasticity::Psi(const Mat3<Real> &f) const -> Real {
  const Mat3<Real> eps = SmallStrain(f);
  const Real trace = eps.trace();
  return lame.mu * eps.squaredNorm() + lame.lambda / 2 * trace * trace;
}

// eps is symmetric, so dPsi/dF = dPsi/deps = 2 mu eps + lambda tr(eps) I.
auto LinearElasticity::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  const Mat3<Real> eps = SmallStrain(f);
  return 2 * lame.mu * eps + lame.lambda * eps.trace() * Mat3<Real>::Identity();
}

// dP = mu (dF + dF^T) + lambda tr(dF) I, so H = 2 mu S + 3 lambda v v^T with
// S the symmetric projector and v the unit volumetric mode.
auto LinearElasticity::Hessian(const Mat3<Real> & /*f*/) const -> Mat9<Real> {
  const Vec9<Real> v = UnitVolumetric();
  return 2 * lame.mu * SymmetricProjector() +
         3 * lame.lambda * v * v.transpose();
}

// The eigensystem of the constant Hessian: the three skew modes have
// eigenvalue 0, the five deviatoric symmetric modes 2 mu, and the volumetric
// mode 2 mu + 3 lambda.
auto LinearElasticity::ClampedHessian(const Mat3<Real> & /*f*/,
                                      const Svd & /*svd*/) const -> Mat9<Real> {
  const Vec9<Real> v = UnitVolumetric();
  const Mat9<Real> volumetric = v * v.transpose();
  return std::max(2 * lame.mu, Real(0)) * (SymmetricProjector() - volumetric) +
         std::max(2 * lame.mu + 3 * lame.lambda, Real(0)) * volumetric;
}

} // namespace sim
