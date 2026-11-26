#pragma once

#include <LibMath.h>

#include <concepts>

namespace sim {

template <int N> using VecN = Eigen::Matrix<Real, N, 1>;
template <int N> using MatN = Eigen::Matrix<Real, N, N>;

// An energy over N stacked vertex DOFs. ClampedHessian projects Hessian onto
// the PSD cone by clamping its negative eigenvalues to 0.
template <typename E, int N>
concept ElementEnergy = requires(const E &e, const VecN<N> &x) {
  { e.Psi(x) } -> std::same_as<Real>;
  { e.Gradient(x) } -> std::same_as<VecN<N>>;
  { e.Hessian(x) } -> std::same_as<MatN<N>>;
  { e.ClampedHessian(x) } -> std::same_as<MatN<N>>;
};

} // namespace sim
