#include "StableNeoHookean.h"

namespace sim {

namespace {

auto SnhLambda(const Lame &lame) -> Real { return lame.lambda + lame.mu; }

// dJ/dF: the cofactor matrix, column i is the cross product of the other two.
auto CofactorOf(const Mat3<Real> &f) -> Mat3<Real> {
  Mat3<Real> g;
  g.col(0) = f.col(1).cross(f.col(2));
  g.col(1) = f.col(2).cross(f.col(0));
  g.col(2) = f.col(0).cross(f.col(1));
  return g;
}

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
  const Mat3<Real> g = CofactorOf(f);
  return mu * f - mu * g + lambda * (j - 1) * g;
}

// H = mu I + l vec(g) vec(g)^T + (l (J - 1) - mu) d^2J/dF^2. With f_i the
// columns of F, the (a, b) 3x3 block of d^2J/dF^2 is the derivative of column a
// of the cofactor in f_b: d(f1 x f2)/df1 = -[f2]x, d(f1 x f2)/df2 = [f1]x, and
// so on around the cycle (Smith et al. 2018, Sec. 4.5).
auto StableNeoHookean::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  const Real mu = lame.mu;
  const Real lambda = SnhLambda(lame);
  const Vec9<Real> g = Vectorize(CofactorOf(f));
  const Real s = lambda * (f.determinant() - 1) - mu;

  Mat9<Real> hessianJ = Mat9<Real>::Zero();
  for (int a = 0; a < 3; ++a) {
    const int b = (a + 1) % 3;
    const int c = (a + 2) % 3;
    const Mat3<Real> cross = CrossProductMatrix(f.col(c));
    hessianJ.block<3, 3>(3 * a, 3 * b) = -cross;
    hessianJ.block<3, 3>(3 * b, 3 * a) = cross;
  }

  return mu * Mat9<Real>::Identity() + lambda * g * g.transpose() +
         s * hessianJ;
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
