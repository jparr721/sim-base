#include "StableNeoHookeanBarrier.h"

#include <limits>

namespace sim {

namespace {

// Psi = mu/2 (|F|^2 - 3) + h(J) with
// h = -mu (J - 1) + l/2 (J - 1)^2 + b(J), and for 0 < J < g, with d = J - g:
//   b   = -l d^2 log(J / g),
//   b'  = -l (2 d log(J / g) + d^2 / J),
//   b'' = -l (2 log(J / g) + 4 d / J - d^2 / J^2).
struct VolumeTerm {
  Real h;
  Real dh;
  Real d2h;
};

auto VolumeTermOf(const Lame &lame, Real j) -> VolumeTerm {
  const Real l = lame.lambda + lame.mu;
  VolumeTerm v{.h = -lame.mu * (j - 1) + l / 2 * (j - 1) * (j - 1),
               .dh = l * (j - 1) - lame.mu,
               .d2h = l};
  constexpr Real g = StableNeoHookeanBarrier::kBarrierJ;
  if (j <= 0) {
    v.h = std::numeric_limits<Real>::infinity();
  } else if (j < g) {
    const Real d = j - g;
    const Real logRatio = std::log(j / g);
    v.h -= l * d * d * logRatio;
    v.dh -= l * (2 * d * logRatio + d * d / j);
    v.d2h -= l * (2 * logRatio + 4 * d / j - d * d / (j * j));
  }
  return v;
}

} // namespace

auto StableNeoHookeanBarrier::Psi(const Mat3<Real> &f) const -> Real {
  return lame.mu / 2 * (f.squaredNorm() - 3) +
         VolumeTermOf(lame, f.determinant()).h;
}

auto StableNeoHookeanBarrier::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  return lame.mu * f + VolumeTermOf(lame, f.determinant()).dh * Cofactor(f);
}

auto StableNeoHookeanBarrier::Hessian(const Mat3<Real> &f) const
    -> Mat9<Real> {
  const VolumeTerm v = VolumeTermOf(lame, f.determinant());
  return StretchVolumeHessian(f, lame.mu, v.dh, v.d2h);
}

// Analytic eigensystem through the mu/2 |F|^2 + h(J) form shared with SNH.
auto StableNeoHookeanBarrier::ClampedHessian(const Mat3<Real> & /*f*/,
                                             const Svd &svd) const
    -> Mat9<Real> {
  const VolumeTerm v = VolumeTermOf(lame, svd.sigma.prod());
  return ClampedStretchVolumeHessian(svd, lame.mu, v.dh, v.d2h);
}

} // namespace sim
