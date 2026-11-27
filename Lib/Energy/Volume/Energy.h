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

struct Lame {
  Real mu;
  Real lambda;

  static auto FromYoungs(Real e, Real nu) -> Lame {
    return {.mu = e / (2 * (1 + nu)),
            .lambda = e * nu / ((1 + nu) * (1 - 2 * nu))};
  }
};

// The clamped Hessian of an isotropic energy Psi(sigma) from its eigensystem
// (Smith et al. 2019, "Analytic Eigensystems for Isotropic Distortion
// Energies"; Dynamic Deformables, "Eigenanalysis"). For the pair (i, j) left
// when index k is removed, the twist matrix U (e_i e_j^T - e_j e_i^T) V^T / sqrt2
// has eigenvalue twist(k) and the flip matrix U (e_i e_j^T + e_j e_i^T) V^T /
// sqrt2 has eigenvalue flip(k). The three scaling modes are U diag(a) V^T for
// each eigenvector a of scaling = d^2 Psi / d sigma^2.
SIM_INLINE auto ClampedIsotropicHessian(const Svd &svd,
                                        const Mat3<Real> &scaling,
                                        const Vec3<Real> &twist,
                                        const Vec3<Real> &flip) -> Mat9<Real> {
  Mat9<Real> h = Mat9<Real>::Zero();
  const auto add = [&](Real eigenvalue, const Mat3<Real> &mode) {
    if (eigenvalue <= 0)
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

} // namespace sim
