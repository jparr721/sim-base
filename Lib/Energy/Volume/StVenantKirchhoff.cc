#include "StVenantKirchhoff.h"

namespace sim {

namespace {

auto GreenStrain(const Mat3<Real> &f) -> Mat3<Real> {
  return (f.transpose() * f - Mat3<Real>::Identity()) / 2;
}

// The second Piola-Kirchhoff stress S = 2 mu E + lambda tr(E) I.
auto Pk2(const Lame &lame, const Mat3<Real> &e) -> Mat3<Real> {
  return 2 * lame.mu * e + lame.lambda * e.trace() * Mat3<Real>::Identity();
}

} // namespace

auto StVenantKirchhoff::Psi(const Mat3<Real> &f) const -> Real {
  const Mat3<Real> e = GreenStrain(f);
  const Real trace = e.trace();
  return lame.mu * e.squaredNorm() + lame.lambda / 2 * trace * trace;
}

auto StVenantKirchhoff::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  return f * Pk2(lame, GreenStrain(f));
}

// dP = dF S + F dS, with dE = (dF^T F + F^T dF) / 2 and dS linear in dE.
// Column 3 b + a is vec(dP) for dF = e_a e_b^T.
auto StVenantKirchhoff::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  const Mat3<Real> s = Pk2(lame, GreenStrain(f));
  Mat9<Real> h;
  for (int b = 0; b < 3; ++b) {
    for (int a = 0; a < 3; ++a) {
      Mat3<Real> df = Mat3<Real>::Zero();
      df(a, b) = 1;
      const Mat3<Real> de = (df.transpose() * f + f.transpose() * df) / 2;
      h.col(3 * b + a) = Vectorize(df * s + f * Pk2(lame, de));
    }
  }
  return h;
}

// In singular values, Psi = mu/4 sum (s_i^2 - 1)^2 + lambda/8 (sum s_i^2 - 3)^2,
// so dPsi/ds_i = mu (s_i^2 - 1) s_i + t s_i with t = lambda/2 (sum s^2 - 3).
// The twist and flip eigenvalues (psi_i +- psi_j) / (s_i +- s_j) reduce to
// mu (s_i^2 -+ s_i s_j + s_j^2 - 1) + t, with no division, and the scaling
// block is d^2 Psi / ds_i ds_j (Smith et al. 2019).
auto StVenantKirchhoff::ClampedHessian(const Mat3<Real> & /*f*/,
                                       const Svd &svd) const -> Mat9<Real> {
  const Real mu = lame.mu;
  const Real lambda = lame.lambda;
  const Vec3<Real> &sigma = svd.sigma;
  const Real t = lambda / 2 * (sigma.squaredNorm() - 3);

  Mat3<Real> scaling = lambda * sigma * sigma.transpose();
  for (int i = 0; i < 3; ++i)
    scaling(i, i) += mu * (3 * sigma(i) * sigma(i) - 1) + t;

  Vec3<Real> twist;
  Vec3<Real> flip;
  for (int k = 0; k < 3; ++k) {
    const Real si = sigma((k + 1) % 3);
    const Real sj = sigma((k + 2) % 3);
    const Real squares = si * si + sj * sj - 1;
    twist(k) = mu * (squares - si * sj) + t;
    flip(k) = mu * (squares + si * sj) + t;
  }
  return ClampedIsotropicHessian(svd, scaling, twist, flip);
}

} // namespace sim
