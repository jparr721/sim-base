#pragma once

#include <LibMath.h>
#include <Solid/Constraints.h>
#include <Solid/System.h>

#include <cmath>
#include <variant>

namespace sim {

// Time integrators over the System concept (Dynamic Deformables, Kim &
// Eberle 2022, "Time Integration"). Each one advances x and v by dt under
// the system's forces plus a constant external force, and never sees an
// energy or a mesh.

// Symplectic Euler: v += dt M^-1 f(x, v), then x += dt v. Explicit, so dt has
// to resolve the stiffest mode.
struct ForwardEuler {};

// Newton on the end-of-step positions, so each step minimizes the incremental
// potential. tolerance bounds the largest force residual on a free DOF.
struct BackwardEuler {
  int maxNewtonIterations = 10;
  Real tolerance = 1e-6;
};

// Newmark's family; beta = 1/4, gamma = 1/2 is the trapezoidal rule.
struct Newmark {
  Real beta = 0.25;
  Real gamma = 0.5;
  int maxNewtonIterations = 10;
  Real tolerance = 1e-6;
};

// Second-order backward differentiation. The first step is BackwardEuler.
struct Bdf2 {
  int maxNewtonIterations = 10;
  Real tolerance = 1e-6;
};

// Static equilibrium under the external force: velocity is always zero.
struct Quasistatic {
  int maxNewtonIterations = 50;
  Real tolerance = 1e-6;
};

using IntegratorMethod =
    std::variant<ForwardEuler, BackwardEuler, Newmark, Bdf2, Quasistatic>;

// No default constructor, so a body that leaves out its integrator does not
// compile.
struct Integrator : IntegratorMethod {
  using IntegratorMethod::IntegratorMethod;
  Integrator() = delete;
};

// A body's DOFs as its integrator advances them. The history fields start
// empty and the methods that need them fill them on their first step.
struct IntegratorState {
  Vec<Real> x;
  Vec<Real> v;
  // Bdf2: x and v one step back.
  Vec<Real> xPrevious;
  Vec<Real> vPrevious;
  // Newmark: the acceleration at x.
  Vec<Real> a;
};

// A step that did not converge still leaves a finite state, the last Newton
// iterate the line search accepted.
struct StepReport {
  bool converged = true;
  int newtonIterations = 0;
  int cgIterations = 0;
};

namespace detail {

// The implicit methods' end-of-step acceleration and velocity as affine
// functions of the end-of-step positions x: a(x) = alphaA (x - xA) and
// v(x) = alphaV (x - xV). Quasistatic has both alphas zero.
struct AffineStep {
  Real alphaA;
  Vec<Real> xA;
  Real alphaV;
  Vec<Real> xV;
};

struct CgSolve {
  Vec<Real> dx;
  int iterations;
};

// Solves (S A S + I - S) dx = b by Jacobi-preconditioned conjugate gradients,
// with S the constraint filter and b already filtered. A must be SPD on the
// free DOFs. Single-threaded, so the result is the same on every run.
auto SolveFiltered(const SparseMat<Real> &a, const Constraints &constraints,
                   const Vec<Real> &b) -> CgSolve;

// Zeroes the constrained DOFs of y.
void Project(const Constraints &constraints, Vec<Real> &y);

auto LumpedInverse(const SparseMat<Real> &mass) -> Vec<Real>;

// Solves the method's force residual g(x) = M a(x) - Forces(x, v(x)) -
// external = 0 by Newton from `guess`. Iterate x_k minimizes along its step
// Phi_k(y) = alphaA/2 |y - xA|^2_M + alphaV/2 |y - xV|^2_{D_k} + E(y) -
// external . y, with D_k = DampingMatrix(x_k). The D_k term is the Rayleigh
// dissipation potential 1/2 v^T D v scaled to the method. Every damping force
// is linear in v, Forces(x, v) = Forces(x, 0) - D(x) v, so
// grad Phi_k(x_k) = g(x_k), and a decrease of Phi_k is progress on g. Each
// Newton step solves (alphaA M + K + alphaV D_k) dx = -g with the clamped K,
// then backtracks on Phi_k (Armijo). The Jacobian leaves out how D(x) v
// changes with x, which is zero for contact damping and drag, so Newton is
// exact for them and converges linearly under material damping.
template <System S>
auto Newton(const S &system, const AffineStep &map, const Vec<Real> &external,
            const Vec<Real> &guess, int maxIterations, Real tolerance,
            Vec<Real> &x) -> StepReport {
  const Constraints &constraints = system.Constraints();
  const SparseMat<Real> &mass = system.Mass();
  SparseMat<Real> damping = system.DampingMatrix(guess);
  // Phi_k, and the sum of its terms' magnitudes, which scales its round-off.
  struct Potential {
    Real value;
    Real magnitude;
    Real energy;
  };
  const auto phiWith = [&](const Vec<Real> &y, Real energy) -> Potential {
    const Vec<Real> dA = y - map.xA;
    const Vec<Real> dV = y - map.xV;
    const Real inertia = map.alphaA / 2 * dA.dot(mass * dA);
    const Real dissipation = map.alphaV / 2 * dV.dot(damping * dV);
    const Real work = external.dot(y);
    return {.value = inertia + dissipation + energy - work,
            .magnitude =
                inertia + dissipation + std::abs(energy) + std::abs(work),
            .energy = energy};
  };
  const auto phi = [&](const Vec<Real> &y) -> Potential {
    return phiWith(y, system.Energy(y));
  };
  const auto residual = [&](const Vec<Real> &y) -> Vec<Real> {
    Vec<Real> g = map.alphaA * (mass * (y - map.xA)) -
                  system.Forces(y, map.alphaV * (y - map.xV)) - external;
    Project(constraints, g);
    return g;
  };

  x = guess;
  StepReport report{.converged = false};
  Vec<Real> g = residual(x);
  Potential phiX = phi(x);
  for (;; ++report.newtonIterations) {
    if (!g.allFinite() || !std::isfinite(phiX.value)) {
      return report;
    }
    if (g.lpNorm<Eigen::Infinity>() <= tolerance) {
      report.converged = true;
      return report;
    }
    if (report.newtonIterations == maxIterations) {
      return report;
    }
    if (report.newtonIterations > 0 && map.alphaV != 0) {
      damping = system.DampingMatrix(x);
      phiX = phiWith(x, phiX.energy);
    }
    const SparseMat<Real> jacobian = map.alphaA * mass +
                                     system.Stiffness(x, true) +
                                     map.alphaV * damping;
    const CgSolve solve = SolveFiltered(jacobian, constraints, -g);
    report.cgIterations += solve.iterations;
    const Real slope = g.dot(solve.dx);
    if (!(slope < 0)) {
      return report;
    }
    // A trial passes on a sufficient decrease of Phi (Armijo). Near the
    // solution Phi's changes sink below its round-off, which cancellation
    // inside Psi makes far larger than eps |Phi|, so a trial also passes when
    // Phi stays within a round-off band and |g| drops enough.
    const Real gNorm = g.norm();
    const Real band = 1e-10 * phiX.magnitude;
    for (Real alpha = 1;; alpha /= 2) {
      if (alpha < 1e-9) {
        return report;
      }
      Vec<Real> trial = x + alpha * solve.dx;
      Vec<Real> gTrial = residual(trial);
      const Potential phiTrial = phi(trial);
      if (phiTrial.value <= phiX.value + 1e-4 * alpha * slope ||
          (phiTrial.value <= phiX.value + band &&
           gTrial.norm() <= (1 - 1e-4 * alpha) * gNorm)) {
        x = std::move(trial);
        g = std::move(gTrial);
        phiX = phiTrial;
        break;
      }
    }
  }
}

} // namespace detail

template <System S>
auto Step(const ForwardEuler & /*method*/, const S &system,
          IntegratorState &state, const Vec<Real> &external, Real dt)
    -> StepReport {
  const SparseMat<Real> massInverse = ConstructDiagonalSparseMatrix<Real>(
      Vec<Real>(system.Mass().diagonal()).cwiseInverse());
  const SparseMat<Real> filter = system.Constraints().Filter();
  const Vec<Real> forces = system.Forces(state.x, state.v);
  // dx = S (dt^2 M^-1 f + dt v), then v = S dx / dt.
  Vec<Real> dx = (dt * dt) * massInverse * (forces + external) + dt * state.v;
  dx = filter * dx;
  state.x = state.x + dx;
  state.v = filter * Vec<Real>(dx / dt);
  return {};
}

// v(x) = (x - x_n) / dt and a(x) = (v(x) - v_n) / dt.
template <System S>
auto Step(const BackwardEuler &method, const S &system, IntegratorState &state,
          const Vec<Real> &external, Real dt) -> StepReport {
  const Vec<Real> inertial = state.x + dt * state.v;
  const detail::AffineStep map{.alphaA = 1 / (dt * dt),
                               .xA = inertial,
                               .alphaV = 1 / dt,
                               .xV = state.x};
  Vec<Real> x;
  const StepReport report =
      detail::Newton(system, map, external, inertial,
                     method.maxNewtonIterations, method.tolerance, x);
  state.v = map.alphaV * (x - map.xV);
  detail::Project(system.Constraints(), state.v);
  state.x = std::move(x);
  return report;
}

// x_{n+1} = x_n + dt v_n + dt^2 ((1/2 - beta) a_n + beta a_{n+1}) and
// v_{n+1} = v_n + dt ((1 - gamma) a_n + gamma a_{n+1}), solved for x_{n+1}.
template <System S>
auto Step(const Newmark &method, const S &system, IntegratorState &state,
          const Vec<Real> &external, Real dt) -> StepReport {
  if (state.a.size() == 0) {
    state.a = detail::LumpedInverse(system.Mass()).cwiseProduct(
        system.Forces(state.x, state.v) + external);
    detail::Project(system.Constraints(), state.a);
  }
  const Real beta = method.beta;
  const Real gamma = method.gamma;
  const Vec<Real> predicted =
      state.x + dt * state.v + (dt * dt * (0.5 - beta)) * state.a;
  const Vec<Real> velocityPredicted = state.v + (dt * (1 - gamma)) * state.a;
  const Real alphaA = 1 / (beta * dt * dt);
  const Real alphaV = gamma / (beta * dt);
  const detail::AffineStep map{.alphaA = alphaA,
                               .xA = predicted,
                               .alphaV = alphaV,
                               .xV = predicted - velocityPredicted / alphaV};
  Vec<Real> x;
  const StepReport report = detail::Newton(
      system, map, external, Vec<Real>(state.x + dt * state.v),
      method.maxNewtonIterations, method.tolerance, x);
  state.a = alphaA * (x - map.xA);
  state.v = alphaV * (x - map.xV);
  detail::Project(system.Constraints(), state.a);
  detail::Project(system.Constraints(), state.v);
  state.x = std::move(x);
  return report;
}

// x_{n+1} = (4 x_n - x_{n-1}) / 3 + (2/3) dt v_{n+1}, and the same rule for
// v_{n+1} with a_{n+1}, solved for x_{n+1}.
template <System S>
auto Step(const Bdf2 &method, const S &system, IntegratorState &state,
          const Vec<Real> &external, Real dt) -> StepReport {
  if (state.xPrevious.size() == 0) {
    state.xPrevious = state.x;
    state.vPrevious = state.v;
    return Step(BackwardEuler{.maxNewtonIterations = method.maxNewtonIterations,
                              .tolerance = method.tolerance},
                system, state, external, dt);
  }
  const Vec<Real> xV = (4 * state.x - state.xPrevious) / 3;
  const detail::AffineStep map{
      .alphaA = 9 / (4 * dt * dt),
      .xA = xV + (2 * dt / 9) * (4 * state.v - state.vPrevious),
      .alphaV = 3 / (2 * dt),
      .xV = xV};
  Vec<Real> x;
  const StepReport report = detail::Newton(
      system, map, external, Vec<Real>(state.x + dt * state.v),
      method.maxNewtonIterations, method.tolerance, x);
  state.xPrevious = std::move(state.x);
  state.vPrevious = std::move(state.v);
  state.v = map.alphaV * (x - map.xV);
  detail::Project(system.Constraints(), state.v);
  state.x = std::move(x);
  return report;
}

// Minimizes E(x) - external . x from the current positions.
template <System S>
auto Step(const Quasistatic &method, const S &system, IntegratorState &state,
          const Vec<Real> &external, Real /*dt*/) -> StepReport {
  const detail::AffineStep map{.alphaA = 0,
                               .xA = Vec<Real>::Zero(system.Dofs()),
                               .alphaV = 0,
                               .xV = Vec<Real>::Zero(system.Dofs())};
  Vec<Real> x;
  const StepReport report =
      detail::Newton(system, map, external, state.x,
                     method.maxNewtonIterations, method.tolerance, x);
  state.x = std::move(x);
  state.v.setZero();
  return report;
}

} // namespace sim
