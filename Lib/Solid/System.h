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

// A System whose damping is not a fixed matrix over a step, such as a damping
// energy's, also gives the implicit step's dissipation. Over a step whose
// velocity is v(x) = alphaV (x - xV), Dissipation(x, xV, alphaV) is the
// dissipation W(x) the step does, and StepForces(x, xV, alphaV) is Forces
// with each damping term replaced by its step form, so that
// StepForces = -d(Energy + Dissipation)/dx. A static solve has alphaV = 0 and
// no velocity, and both leave damping out. An implicit integrator gives any
// other System the W of its damping matrix at xV, held fixed over the step.
template <typename S>
concept StepDissipativeSystem =
    System<S> && requires(const S &s, const Vec<Real> &x, Real alphaV) {
      { s.Dissipation(x, x, alphaV) } -> std::same_as<Real>;
      { s.StepForces(x, x, alphaV) } -> std::same_as<Vec<Real>>;
    };

} // namespace sim
