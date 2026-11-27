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

// Each implicit step solves for the end-of-step positions by minimizing one
// incremental potential, fixed for the whole step:
//
//   Phi(y) = alphaA/2 |y - xA|^2_M + E(y) - external . y + W(y).
//
// W is the dissipation the step does. Over a step of length h = 1 / alphaV,
// with v(y) = (y - xV) / h, it is h times the dissipation potential at the
// step's mean rates. Damping given as a matrix D (contacts, drag, Rayleigh)
// adds h v^T D v / 2 = alphaV/2 |y - xV|^2_D, with D taken at xV. Green
// damping adds, over tets, V h Psi_d at the mean strain rate
// (E(F(y)) - E(F(xV))) / h, which is GreenDamping::StepPsi. Per method:
//
//   BackwardEuler  alphaA = 1 / dt^2,        h = dt,             xV = x_n.
//   Newmark        alphaA = 1 / (beta dt^2), h = beta dt / gamma,
//                  xV = xA - h (v_n + (1 - gamma) dt a_n).
//   Bdf2           alphaA = 9 / (4 dt^2),    h = 2 dt / 3,
//                  xV = (4 x_n - x_{n-1}) / 3.
//   Quasistatic    alphaA = 0, and no W.
//
// xA is each Step's inertial target. grad Phi is the residual Newton drives
// to zero: M a(y) minus the step's forces, which are the elastic and contact
// forces, -D v(y), and Green damping's StepPk1 stress
// 2 mu F (E(F) - E(F(xV))) / h. As h shrinks that stress tends to the rate
// form 2 mu F Edot, and unlike the rate form it leaves a rigid step undamped
// however far it turns. A System that is not a StepDissipativeSystem gets
// D = DampingMatrix(xV) for all of its damping.
template <System S> class IncrementalPotential {
public:
  // Phi, and the sum of its terms' magnitudes, which scales its round-off.
  struct Value {
    Real value;
    Real magnitude;
  };

  // system, map and external must outlive the potential.
  IncrementalPotential(const S &system, const AffineStep &map,
                       const Vec<Real> &external)
      : system(system), map(map), external(external) {
    if constexpr (!StepDissipativeSystem<S>) {
      fixedDamping = map.alphaV != 0
                         ? system.DampingMatrix(map.xV)
                         : SparseMat<Real>(system.Dofs(), system.Dofs());
    }
  }

  auto operator()(const Vec<Real> &y) const -> Value {
    const Vec<Real> dA = y - map.xA;
    const Real inertia = map.alphaA / 2 * dA.dot(system.Mass() * dA);
    const Real dissipation = Dissipation(y);
    const Real energy = system.Energy(y);
    const Real work = external.dot(y);
    return {.value = inertia + dissipation + energy - work,
            .magnitude =
                inertia + dissipation + std::abs(energy) + std::abs(work)};
  }

  // grad Phi, with the constrained DOFs zeroed.
  auto Gradient(const Vec<Real> &y) const -> Vec<Real> {
    Vec<Real> g =
        map.alphaA * (system.Mass() * (y - map.xA)) - Forces(y) - external;
    Project(system.Constraints(), g);
    return g;
  }

  // alphaA M + K + alphaV D(y), with K clamped. Under Green damping it leaves
  // out the indefinite term 2 mu dF (E(F) - E(F(xV))) / h, so Newton
  // converges linearly there.
  auto Hessian(const Vec<Real> &y) const -> SparseMat<Real> {
    return map.alphaA * system.Mass() + system.Stiffness(y, true) +
           map.alphaV * system.DampingMatrix(y);
  }

  auto Constraints() const -> const sim::Constraints & {
    return system.Constraints();
  }

private:
  const S &system;
  const AffineStep &map;
  const Vec<Real> &external;
  SparseMat<Real> fixedDamping;

  auto Dissipation(const Vec<Real> &y) const -> Real {
    if constexpr (StepDissipativeSystem<S>) {
      return system.Dissipation(y, map.xV, map.alphaV);
    } else {
      const Vec<Real> dV = y - map.xV;
      return map.alphaV / 2 * dV.dot(fixedDamping * dV);
    }
  }

  auto Forces(const Vec<Real> &y) const -> Vec<Real> {
    if constexpr (StepDissipativeSystem<S>) {
      return system.StepForces(y, map.xV, map.alphaV);
    } else {
      Vec<Real> forces = system.Forces(y, Vec<Real>::Zero(system.Dofs()));
      forces -= fixedDamping * Vec<Real>(map.alphaV * (y - map.xV));
      return forces;
    }
  }
};

// Minimizes phi by Newton from `guess`, until no free DOF's residual exceeds
// tolerance. Each iteration solves Hessian dx = -Gradient, then backtracks
// on phi (Armijo).
template <System S>
auto Newton(const IncrementalPotential<S> &phi, const Vec<Real> &guess,
            int maxIterations, Real tolerance, Vec<Real> &x) -> StepReport {
  using Value = typename IncrementalPotential<S>::Value;
  x = guess;
  StepReport report{.converged = false};
  Vec<Real> g = phi.Gradient(x);
  Value phiX = phi(x);
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
    const CgSolve solve = SolveFiltered(phi.Hessian(x), phi.Constraints(), -g);
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
      Vec<Real> gTrial = phi.Gradient(trial);
      const Value phiTrial = phi(trial);
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
  const SparseMat<Real> massInverse =
      ConstructDiagonalSparseMatrix<Real>(detail::LumpedInverse(system.Mass()));
  const SparseMat<Real> filter = system.Constraints().Filter();
  const Vec<Real> forces = system.Forces(state.x, state.v);
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
      detail::Newton(detail::IncrementalPotential(system, map, external),
                     inertial, method.maxNewtonIterations, method.tolerance, x);
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
      detail::IncrementalPotential(system, map, external),
      Vec<Real>(state.x + dt * state.v), method.maxNewtonIterations,
      method.tolerance, x);
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
      detail::IncrementalPotential(system, map, external),
      Vec<Real>(state.x + dt * state.v), method.maxNewtonIterations,
      method.tolerance, x);
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
      detail::Newton(detail::IncrementalPotential(system, map, external),
                     state.x, method.maxNewtonIterations, method.tolerance, x);
  state.x = std::move(x);
  state.v.setZero();
  return report;
}

} // namespace sim
