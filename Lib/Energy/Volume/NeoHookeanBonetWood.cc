#include "NeoHookeanBonetWood.h"

namespace sim {

namespace {

// The guarded log L(J) and its first two derivatives.
struct GuardedLog {
  Real value;
  Real d1;
  Real d2;
};

auto GuardedLogOf(Real j) -> GuardedLog {
  constexpr Real g = NeoHookeanBonetWood::kGuardJ;
  if (j >= g)
    return {.value = std::log(j), .d1 = 1 / j, .d2 = -1 / (j * j)};
  const Real dj = j - g;
  return {.value = std::log(g) + dj / g - dj * dj / (2 * g * g),
          .d1 = 1 / g - dj / (g * g),
          .d2 = -1 / (g * g)};
}

// Psi = mu/2 (|F|^2 - 3) + h(J) with h = -mu L + lambda/2 L^2, so
// h' = (lambda L - mu) L' and h'' = lambda L'^2 + (lambda L - mu) L''.
struct VolumeTerm {
  Real h;
  Real dh;
  Real d2h;
};

auto VolumeTermOf(const Lame &lame, Real j) -> VolumeTerm {
  const GuardedLog l = GuardedLogOf(j);
  const Real slope = lame.lambda * l.value - lame.mu;
  return {.h = -lame.mu * l.value + lame.lambda / 2 * l.value * l.value,
          .dh = slope * l.d1,
          .d2h = lame.lambda * l.d1 * l.d1 + slope * l.d2};
}

} // namespace

auto NeoHookeanBonetWood::Psi(const Mat3<Real> &f) const -> Real {
  return lame.mu / 2 * (f.squaredNorm() - 3) +
         VolumeTermOf(lame, f.determinant()).h;
}

// P = mu F + h'(J) cof F. For J >= kGuardJ, h' cof F = (lambda log J - mu)
// F^-T, the textbook mu (F - F^-T) + lambda log J F^-T.
auto NeoHookeanBonetWood::Pk1(const Mat3<Real> &f) const -> Mat3<Real> {
  return lame.mu * f + VolumeTermOf(lame, f.determinant()).dh * Cofactor(f);
}

auto NeoHookeanBonetWood::Hessian(const Mat3<Real> &f) const -> Mat9<Real> {
  const VolumeTerm v = VolumeTermOf(lame, f.determinant());
  return StretchVolumeHessian(f, lame.mu, v.dh, v.d2h);
}

// Analytic eigensystem (Smith et al. 2019, "Neo-Hookean"), through the
// mu/2 |F|^2 + h(J) form shared with Stable Neo-Hookean.
auto NeoHookeanBonetWood::ClampedHessian(const Mat3<Real> & /*f*/,
                                         const Svd &svd) const -> Mat9<Real> {
  const VolumeTerm v = VolumeTermOf(lame, svd.sigma.prod());
  return ClampedStretchVolumeHessian(svd, lame.mu, v.dh, v.d2h);
}

} // namespace sim
