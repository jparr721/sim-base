#pragma once

#include <LibMath.h>

#include <algorithm>
#include <cmath>
#include <concepts>

namespace sim {

// vec(F) stacks the columns of F, so vec(F)(3 j + i) = F(i, j). Every Mat9 in
// this module is a matrix over vec(F): Hessian(f) is d^2 Psi / d vec(F)^2.
SIM_INLINE auto Vectorize(const Mat3<Real> &m) -> Vec9<Real> {
  return Eigen::Map<const Vec9<Real>>(m.data());
}

// A hyperelastic energy density of the deformation gradient F. ClampedHessian
// is Hessian projected onto the PSD cone (negative eigenvalues set to 0),
// built from the energy's analytic eigensystem at svd = RotationVariantSvd(f).
template <typename E>
concept Energy = requires(const E &e, const Mat3<Real> &f, const Svd &svd) {
  { e.Psi(f) } -> std::same_as<Real>;
  { e.Pk1(f) } -> std::same_as<Mat3<Real>>;
  { e.Hessian(f) } -> std::same_as<Mat9<Real>>;
  { e.ClampedHessian(f, svd) } -> std::same_as<Mat9<Real>>;
};

// A damping energy density of F and its rate Fdot: Psi(F, Fdot) is a
// Rayleigh dissipation potential, so the damping force on a tet is
// -V (dF/dx)^T vec(Pk1) with Pk1 = dPsi/dFdot, and Hessian is
// d^2 Psi / d vec(Fdot)^2. Psi is quadratic in Fdot, so the force is linear
// in the velocities.
template <typename D>
concept DampingEnergy =
    requires(const D &d, const Mat3<Real> &f, const Mat3<Real> &fdot) {
      { d.Psi(f, fdot) } -> std::same_as<Real>;
      { d.Pk1(f, fdot) } -> std::same_as<Mat3<Real>>;
      { d.Hessian(f, fdot) } -> std::same_as<Mat9<Real>>;
    };

struct Lame {
  Real mu;
  Real lambda;

  static auto FromYoungs(Real e, Real nu) -> Lame {
    return {.mu = e / (2 * (1 + nu)),
            .lambda = e * nu / ((1 + nu) * (1 - 2 * nu))};
  }
};

// The Hessian of an isotropic energy Psi(sigma) from its eigensystem
// (Smith et al. 2019, "Analytic Eigensystems for Isotropic Distortion
// Energies"; Dynamic Deformables, "Eigenanalysis"). For the pair (i, j) left
// when index k is removed, the twist matrix U (e_i e_j^T - e_j e_i^T) V^T / sqrt2
// has eigenvalue twist(k) and the flip matrix U (e_i e_j^T + e_j e_i^T) V^T /
// sqrt2 has eigenvalue flip(k). The three scaling modes are U diag(a) V^T for
// each eigenvector a of scaling = d^2 Psi / d sigma^2. clamped drops every
// mode whose eigenvalue is not positive.
SIM_INLINE auto IsotropicHessian(const Svd &svd, const Mat3<Real> &scaling,
                                 const Vec3<Real> &twist,
                                 const Vec3<Real> &flip, bool clamped)
    -> Mat9<Real> {
  Mat9<Real> h = Mat9<Real>::Zero();
  const auto add = [&](Real eigenvalue, const Mat3<Real> &mode) {
    if (clamped && eigenvalue <= 0)
      return;
    const Vec9<Real> q = Vectorize(svd.u * mode * svd.v.transpose());
    h += eigenvalue * q * q.transpose();
  };

  const Eigen::SelfAdjointEigenSolver<Mat3<Real>> scalingModes(scaling);
  for (int m = 0; m < 3; ++m)
    add(scalingModes.eigenvalues()(m),
        Mat3<Real>(scalingModes.eigenvectors().col(m).asDiagonal()));

  const Real invSqrt2 = 1 / std::sqrt(Real(2));
  for (int k = 0; k < 3; ++k) {
    const int i = (k + 1) % 3;
    const int j = (k + 2) % 3;
    Mat3<Real> antisymmetric = Mat3<Real>::Zero();
    antisymmetric(i, j) = invSqrt2;
    antisymmetric(j, i) = -invSqrt2;
    Mat3<Real> symmetric = Mat3<Real>::Zero();
    symmetric(i, j) = invSqrt2;
    symmetric(j, i) = invSqrt2;
    add(twist(k), antisymmetric);
    add(flip(k), symmetric);
  }
  return h;
}

SIM_INLINE auto ClampedIsotropicHessian(const Svd &svd,
                                        const Mat3<Real> &scaling,
                                        const Vec3<Real> &twist,
                                        const Vec3<Real> &flip) -> Mat9<Real> {
  return IsotropicHessian(svd, scaling, twist, flip, true);
}

// dJ/dF for J = det F: the cofactor matrix, column i is the cross product of
// the other two.
SIM_INLINE auto Cofactor(const Mat3<Real> &f) -> Mat3<Real> {
  Mat3<Real> g;
  g.col(0) = f.col(1).cross(f.col(2));
  g.col(1) = f.col(2).cross(f.col(0));
  g.col(2) = f.col(0).cross(f.col(1));
  return g;
}

// d^2 J / d vec(F)^2. With f_i the columns of F, the (a, b) 3x3 block is the
// derivative of cofactor column a in f_b: d(f1 x f2)/df1 = -[f2]x,
// d(f1 x f2)/df2 = [f1]x, and so on around the cycle (Smith et al. 2018,
// Sec. 4.5).
SIM_INLINE auto DeterminantHessian(const Mat3<Real> &f) -> Mat9<Real> {
  Mat9<Real> h = Mat9<Real>::Zero();
  for (int a = 0; a < 3; ++a) {
    const int b = (a + 1) % 3;
    const Mat3<Real> cross = CrossProductMatrix(f.col((a + 2) % 3));
    h.block<3, 3>(3 * a, 3 * b) = -cross;
    h.block<3, 3>(3 * b, 3 * a) = cross;
  }
  return h;
}

// Energies of the form Psi = mu/2 |F|^2 + h(J), the Neo-Hookean family, given
// dh = h'(J) and d2h = h''(J). P = mu F + h' cof F, and the Hessian is
// mu I + h'' vec(cof F) vec(cof F)^T + h' d^2J/dF^2.
SIM_INLINE auto StretchVolumeHessian(const Mat3<Real> &f, Real mu, Real dh,
                                     Real d2h) -> Mat9<Real> {
  const Vec9<Real> g = Vectorize(Cofactor(f));
  return mu * Mat9<Real>::Identity() + d2h * g * g.transpose() +
         dh * DeterminantHessian(f);
}

// The clamped Hessian of mu/2 |F|^2 + h(J). In singular values,
// dPsi/ds_i = mu s_i + h' s_j s_k, so the twist and flip eigenvalues
// (psi_i +- psi_j) / (s_i +- s_j) are mu +- h' s_k, and the scaling block is
// d^2 Psi / ds_i ds_j = mu delta_ij + h'' (J / s_i)(J / s_j) + h' s_k (i != j)
// (Smith et al. 2019; Smith et al. 2018, Sec. 4.4).
SIM_INLINE auto ClampedStretchVolumeHessian(const Svd &svd, Real mu, Real dh,
                                            Real d2h) -> Mat9<Real> {
  const Vec3<Real> &s = svd.sigma;
  const Vec3<Real> others(s(1) * s(2), s(0) * s(2), s(0) * s(1));
  Mat3<Real> scaling = d2h * others * others.transpose();
  for (int k = 0; k < 3; ++k) {
    const int i = (k + 1) % 3;
    const int j = (k + 2) % 3;
    scaling(i, j) += dh * s(k);
    scaling(j, i) += dh * s(k);
    scaling(k, k) += mu;
  }
  const Vec3<Real> muVec = Vec3<Real>::Constant(mu);
  return ClampedIsotropicHessian(svd, scaling, muVec + dh * s, muVec - dh * s);
}

// Fiber energies Psi(I5) of I5 = |F a|^2 for a unit fiber a (Kim et al.
// 2019, "Anisotropic Elasticity for Inversion-Safety and Element
// Rehabilitation"), given dpsi = Psi'(I5): dI5/dF = 2 F a a^T.
SIM_INLINE auto FiberPk1(const Mat3<Real> &f, const Vec3<Real> &a, Real dpsi)
    -> Mat3<Real> {
  return 2 * dpsi * (f * a) * a.transpose();
}

// The Hessian of Psi(I5) is 2 Psi' (a a^T kron I) + 4 Psi'' g g^T with
// g = vec(F a a^T). Its eigensystem (Kim et al. 2019): the range of
// a a^T kron I is the three matrices b a^T. On g / sqrt(I5) the eigenvalue is
// 2 Psi' + 4 I5 Psi'', on the two b a^T with b orthogonal to F a it is
// 2 Psi', and the other six eigenvalues are 0. clamped drops the modes whose
// eigenvalue is not positive.
SIM_INLINE auto FiberHessian(const Mat3<Real> &f, const Vec3<Real> &a,
                             Real dpsi, Real d2psi, bool clamped)
    -> Mat9<Real> {
  Mat9<Real> span = Mat9<Real>::Zero();
  for (int j = 0; j < 3; ++j) {
    for (int l = 0; l < 3; ++l)
      span.block<3, 3>(3 * j, 3 * l).diagonal().setConstant(a(j) * a(l));
  }
  const Vec9<Real> g = Vectorize((f * a) * a.transpose());
  const Real i5 = g.squaredNorm();
  if (!clamped)
    return 2 * dpsi * span + 4 * d2psi * g * g.transpose();

  const Real across = std::max(2 * dpsi, Real(0));
  if (i5 == 0)
    return across * span;
  const Mat9<Real> along = g * g.transpose() / i5;
  return std::max(2 * dpsi + 4 * i5 * d2psi, Real(0)) * along +
         across * (span - along);
}

} // namespace sim
