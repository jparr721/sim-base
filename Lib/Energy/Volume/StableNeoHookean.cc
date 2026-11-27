#include "StableNeoHookean.h"

namespace sim {

namespace {

auto SnhLambda(const Lame &lame) -> Real { return lame.lambda + lame.mu; }

} // namespace

auto StableNeoHookean::Psi(const Mat3<Real> &f) const -> Real {
  const Real mu = lame.mu;
  const Real lambda = SnhLambda(lame);
  const Real jMinus1 = f.determinant() - 1;
  return mu / 2 * (f.squaredNorm() - 3) - mu * jMinus1 +
         lambda / 2 * jMinus1 * jMinus1;
}

// P = mu F + (l (J - 1) - mu) dJ/dF. Written term by term, not factored,
// because the bunny-drop golden pins this rounding bit for bit.
auto StableNeoHookean::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  const Real mu = lame.mu;
  const Real lambda = SnhLambda(lame);
  const Real j = f.determinant();
  const Mat3<Real> g = Cofactor(f);
  return mu * f - mu * g + lambda * (j - 1) * g;
}

// H = mu I + l vec(g) vec(g)^T + (l (J - 1) - mu) d^2J/dF^2. With f_i the
// columns of F, the (a, b) 3x3 block of d^2J/dF^2 is the derivative of column a
// of the cofactor in f_b: d(f1 x f2)/df1 = -[f2]x, d(f1 x f2)/df2 = [f1]x, and
// so on around the cycle (Smith et al. 2018, Sec. 4.5).
auto StableNeoHookean::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  const Real mu = lame.mu;
  const Real lambda = SnhLambda(lame);
  return StretchVolumeHessian(f, mu, lambda * (f.determinant() - 1) - mu,
                              lambda);
}

// In singular values, Psi = mu/2 (sum s_i^2 - 3) - mu (J - 1) + l/2 (J - 1)^2
// with J = s0 s1 s2, so dPsi/ds_i = mu s_i + r s_j s_k for r = l (J - 1) - mu.
// The twist and flip eigenvalues (psi_i +- psi_j) / (s_i +- s_j) reduce to
// mu +- r s_k (Smith et al. 2018, Sec. 4.4), and the scaling block is
// d^2 Psi / ds_i ds_j.
auto StableNeoHookean::ClampedHessian(const Mat3<Real> & /*f*/,
                                      const Svd &svd) const -> Mat9<Real> {
  const Real mu = lame.mu;
  const Real lambda = SnhLambda(lame);
  const Vec3<Real> &sigma = svd.sigma;
  const Real r = lambda * (sigma.prod() - 1) - mu;

  Mat3<Real> scaling;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      // The product of the singular values other than i and j.
      Vec3<Real> others = sigma;
      others(i) = 1;
      others(j) = 1;
      if (i == j) {
        scaling(i, i) = mu + lambda * others.prod() * others.prod();
      } else {
        const Real sk = others.prod();
        scaling(i, j) = sk * r + lambda * sigma(i) * sigma(j) * sk * sk;
      }
    }
  }

  const Vec3<Real> muVec = Vec3<Real>::Constant(mu);
  return ClampedIsotropicHessian(svd, scaling, muVec + r * sigma,
                                 muVec - r * sigma);
}

} // namespace sim
