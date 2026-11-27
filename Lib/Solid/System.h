#pragma once

#include <LibMath.h>
#include <Solid/Constraints.h>

#include <concepts>

namespace sim {

// What an integrator needs from a body, and nothing about its energies or
// mesh. Over stacked DOFs x and velocities v: Energy is the potential E(x),
// Forces is -dE/dx plus damping, Stiffness is d^2E/dx^2 = -dForces/dx
// (clamped = true projects each element block onto the PSD cone), and
// DampingMatrix is -dForces/dv. Damping is linear in v, so
// Forces(x, v) = Forces(x, 0) - DampingMatrix(x) v.
template <typename S>
concept System = requires(const S &s, const Vec<Real> &x, const Vec<Real> &v) {
  { s.Dofs() } -> std::same_as<int>;
  { s.Mass() } -> std::same_as<const SparseMat<Real> &>;
  { s.Energy(x) } -> std::same_as<Real>;
  { s.Forces(x, v) } -> std::same_as<Vec<Real>>;
  { s.Stiffness(x, true) } -> std::same_as<SparseMat<Real>>;
  { s.DampingMatrix(x) } -> std::same_as<SparseMat<Real>>;
  { s.Constraints() } -> std::same_as<const Constraints &>;
};

} // namespace sim
