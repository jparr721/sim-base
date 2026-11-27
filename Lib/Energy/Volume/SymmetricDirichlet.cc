#include "SymmetricDirichlet.h"

namespace sim {

namespace {

// dPsi/ds_i = mu/2 (s_i - s_i^-3). Dividing out s_i + s_j and s_i - s_j by
// hand (s_i^-3 +- s_j^-3 factors through them) leaves
//   twist(k) = mu/2 (1 - (s_i^2 - s_i s_j + s_j^2) / (s_i s_j)^3),
//   flip(k)  = mu/2 (1 + (s_i^2 + s_i s_j + s_j^2) / (s_i s_j)^3),
// with no division by a difference, and the scaling block is diagonal,
// mu/2 (1 + 3 s_i^-4) (Smith et al. 2019).
auto SymmetricDirichletHessian(const Lame &lame, const Svd &svd, bool clamped)
    -> Mat9<Real> {
  const Real halfMu = lame.mu / 2;
  const Vec3<Real> &s = svd.sigma;
  Vec3<Real> twist;
  Vec3<Real> flip;
  for (int k = 0; k < 3; ++k) {
    const Real si = s((k + 1) % 3);
    const Real sj = s((k + 2) % 3);
    const Real squares = si * si + sj * sj;
    const Real cube = std::pow(si * sj, 3);
    twist(k) = halfMu * (1 - (squares - si * sj) / cube);
    flip(k) = halfMu * (1 + (squares + si * sj) / cube);
  }
  const Vec3<Real> scaling = halfMu * (1 + 3 * s.array().pow(-4));
  return IsotropicHessian(svd, scaling.asDiagonal(), twist, flip, clamped);
}

} // namespace

auto SymmetricDirichlet::Psi(const Mat3<Real> &f) const -> Real {
  return lame.mu / 4 * (f.squaredNorm() + f.inverse().squaredNorm() - 6);
}

// d|F^-1|^2/dF = -2 F^-T F^-1 F^-T.
auto SymmetricDirichlet::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  const Mat3<Real> inverseT = f.inverse().transpose();
  return lame.mu / 2 * (f - inverseT * inverseT.transpose() * inverseT);
}

auto SymmetricDirichlet::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  return SymmetricDirichletHessian(lame, RotationVariantSvd(f), false);
}

auto SymmetricDirichlet::ClampedHessian(const Mat3<Real> & /*f*/,
                                        const Svd &svd) const -> Mat9<Real> {
  return SymmetricDirichletHessian(lame, svd, true);
}

} // namespace sim
