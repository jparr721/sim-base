#include <Solid/ElasticSystem.h>
#include <Solid/Integrators.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <memory>
#include <string>

using namespace sim;

namespace {

const Vec3<Real> kGravity(0, -0.981, 0);
const Material kSoft = StableNeoHookean{Lame::FromYoungs(5, 0.45)};
const Material kStiff = StableNeoHookean{Lame::FromYoungs(1e4, 0.45)};

// One tet hanging from its top face: vertices 0 to 2 sit at y = 0 and are
// pinned, and vertex 3 hangs below them.
auto HangingTet() -> std::unique_ptr<TetMesh> {
  Mat<Real> v(4, 3);
  v << 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.3, -1, 0.3;
  Mat<int> f(4, 3);
  f << 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3;
  Mat<int> t(1, 4);
  t << 0, 1, 2, 3;
  auto mesh = std::make_unique<TetMesh>(v, f, t);
  for (const int vertex : {0, 1, 2}) {
    mesh->PinVertex(vertex);
  }
  return mesh;
}

// One unpinned tet with a unit right-angle corner at the origin.
auto FreeTet() -> std::unique_ptr<TetMesh> {
  Mat<Real> v(4, 3);
  v << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Mat<int> f(4, 3);
  f << 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3;
  Mat<int> t(1, 4);
  t << 0, 1, 2, 3;
  return std::make_unique<TetMesh>(v, f, t);
}

// The unit-height bunny, hanging by every vertex above y = 0.85 when
// `hanging`.
auto Bunny(bool hanging) -> std::unique_ptr<TetMesh> {
  auto mesh = std::make_unique<TetMesh>(
      Meshes / "bunny.obj",
      [](const Mat<Real> &v, const Mat<int> &) { return v; });
  for (int ii = 0; hanging && ii < mesh->v.rows(); ++ii) {
    if (mesh->v(ii, 1) > 0.85) {
      mesh->PinVertex(ii);
    }
  }
  return mesh;
}

// A body with its system, state and external force, stepped by `method`.
struct Body {
  std::string name;
  std::unique_ptr<TetMesh> mesh;
  std::unique_ptr<Material> material;
  std::unique_ptr<ElasticSystem> system;
  IntegratorState state;
  Vec<Real> external;

  Body(std::string bodyName, std::unique_ptr<TetMesh> tetMesh,
       const Material &bodyMaterial, const Vec3<Real> &gravity)
      : name(std::move(bodyName)), mesh(std::move(tetMesh)),
        material(std::make_unique<Material>(bodyMaterial)),
        system(std::make_unique<ElasticSystem>(*mesh, *material)),
        state{.x = mesh->Positions(),
              .v = Vec<Real>::Zero(mesh->DOFs())},
        external(system->Mass() * gravity.replicate(mesh->v.rows(), 1)) {}

  template <class Method> auto Step(const Method &method, Real dt) -> StepReport {
    return sim::Step(method, *system, state, external, dt);
  }

  // Kinetic plus potential energy, with the external force's potential
  // -external . x.
  auto TotalEnergy() const -> Real {
    return state.v.dot(system->Mass() * state.v) / 2 +
           system->Energy(state.x) - external.dot(state.x);
  }

  // Round-off in TotalEnergy: Psi sums terms as large as (mu + lambda) V
  // that cancel near rest, and the work term is a dot product.
  auto EnergyRoundOff() const -> Real {
    const Lame lame = std::visit([](const auto &e) { return e.lame; }, *material);
    Real volume = 0;
    for (const Real v : mesh->RestVolumes()) {
      volume += v;
    }
    return 16 * std::numeric_limits<Real>::epsilon() *
           ((lame.mu + lame.lambda) * volume +
            external.cwiseAbs().dot(state.x.cwiseAbs()));
  }

  // The largest static force residual on a free DOF.
  auto StaticResidual() const -> Real {
    Vec<Real> g =
        system->Forces(state.x, Vec<Real>::Zero(system->Dofs())) + external;
    detail::Project(system->Constraints(), g);
    return g.lpNorm<Eigen::Infinity>();
  }
};

auto Hanging(const std::string &name, const Material &material = kSoft)
    -> Body {
  return name == "tet" ? Body("tet", HangingTet(), material, kGravity)
                       : Body("bunny", Bunny(true), material, kGravity);
}

// Stiff enough that ForwardEuler at 1/60 is unstable: the soft bunny already
// is, and the tet takes kStiff and a sideways kick so it oscillates.
auto Stiff(const std::string &name) -> Body {
  if (name != "tet") {
    return Hanging(name);
  }
  Body body = Hanging(name, kStiff);
  body.state.v(9) = 0.5;
  return body;
}

// Accuracy checks need Newton's error far below what they measure. Where
// clamping is active the clamped stiffness makes Newton converge linearly,
// so a tight tolerance needs more iterations than the default 10.
constexpr int kIterations = 100;
constexpr Real kTight = 1e-10;

class Integrators : public testing::TestWithParam<std::string> {};

// Stable at a game time step, and dissipative: total energy never rises by
// more than round-off.
TEST_P(Integrators, BackwardEulerIsStableAndDissipative) {
  Body body = Stiff(GetParam());
  constexpr Real dt = 1.0 / 60;
  const BackwardEuler method{.maxNewtonIterations = kIterations,
                             .tolerance = kTight};
  Real energy = body.TotalEnergy();
  const Real initial = energy;
  Real worstRise = 0;
  int maxNewton = 0;
  for (int step = 1; step <= 600; ++step) {
    const StepReport report = body.Step(method, dt);
    ASSERT_TRUE(report.converged) << "step " << step;
    ASSERT_TRUE(body.state.x.allFinite()) << "step " << step;
    maxNewton = std::max(maxNewton, report.newtonIterations);
    const Real next = body.TotalEnergy();
    worstRise = std::max(worstRise, next - energy);
    energy = next;
  }
  const Real roundOff = body.EnergyRoundOff();
  std::cout << "[ measured ] " << body.name << " energy " << initial << " -> "
            << energy << ", worst one-step rise " << worstRise
            << " (round-off bound " << roundOff << "), max Newton "
            << maxNewton << "\n";
  EXPECT_LT(energy, initial);
  EXPECT_LE(worstRise, roundOff);
}

// The control: the explicit method at the same dt is unstable on both bodies.
TEST_P(Integrators, ForwardEulerBlowsUpAtTheSameDt) {
  Body body = Stiff(GetParam());
  constexpr Real dt = 1.0 / 60;
  const Real initial = std::abs(body.TotalEnergy());
  int step = 1;
  for (; step <= 600; ++step) {
    body.Step(ForwardEuler{}, dt);
    if (!body.state.x.allFinite() ||
        std::abs(body.TotalEnergy()) > 1e6 * initial) {
      break;
    }
  }
  std::cout << "[ measured ] " << body.name << " ForwardEuler at 1/60 blew up "
            << "at step " << step << "\n";
  EXPECT_LE(step, 600);
}

// Undamped, contact-free and with no external force, the trapezoidal rule
// keeps the energy of a 5% stretch within 1% over 600 steps. Its energy error
// is second order in dt and grows with how unresolved the stiffest modes
// are: the tet at 1/60 drifts 0.26%, but the bunny's small tets put its
// spectrum far past 1/dt there, and it drifts 2.2% at 1/60, 1.25% over the
// first 2 s, falling to 0.53%, 0.18% and 0.054% as dt halves. So the bunny
// runs at 1/240.
TEST_P(Integrators, NewmarkConservesEnergy) {
  const bool tet = GetParam() == "tet";
  Body body(GetParam(), tet ? FreeTet() : Bunny(false), kSoft,
            Vec3<Real>::Zero());
  const Vec3<Real> centroid = body.mesh->v.colwise().mean();
  for (int ii = 0; ii < body.mesh->v.rows(); ++ii) {
    body.state.x(3 * ii + 1) =
        centroid.y() + 1.05 * (body.state.x(3 * ii + 1) - centroid.y());
  }
  const Real dt = tet ? 1.0 / 60 : 1.0 / 240;
  const Newmark method{.maxNewtonIterations = kIterations, .tolerance = kTight};
  const Real initial = body.TotalEnergy();
  Real drift = 0;
  for (int step = 1; step <= 600; ++step) {
    ASSERT_TRUE(body.Step(method, dt).converged) << "step " << step;
    drift = std::max(drift, std::abs(body.TotalEnergy() - initial));
  }
  std::cout << "[ measured ] " << body.name << " Newmark at dt " << dt
            << ": energy " << initial << ", worst drift "
            << drift / initial * 100 << "%\n";
  EXPECT_GT(initial, 0);
  EXPECT_LE(drift, 0.01 * initial);
}

// Positions at time `end` after a run from rest at dt, with gravity ramped on
// smoothly over the run.
template <class Method>
auto PositionsAt(const std::string &name, const Method &method, Real dt,
                 Real end) -> Vec<Real> {
  Body body = Hanging(name);
  const Vec<Real> gravity = body.external;
  const int steps = static_cast<int>(std::lround(end / dt));
  for (int step = 1; step <= steps; ++step) {
    const Real ramp = (1 - std::cos(std::numbers::pi * step / steps)) / 2;
    body.external = ramp * gravity;
    EXPECT_TRUE(body.Step(method, dt).converged) << "step " << step;
  }
  return body.state.x;
}

// err(dt) / err(dt / 2) against a dt / 16 reference.
template <class Method>
auto ConvergenceRatio(const std::string &name, const Method &method, Real dt,
                      Real end) -> Real {
  const Vec<Real> reference = PositionsAt(name, method, dt / 16, end);
  const Real coarse = (PositionsAt(name, method, dt, end) - reference).norm();
  const Real fine = (PositionsAt(name, method, dt / 2, end) - reference).norm();
  return coarse / fine;
}

// The load is ramped because switching gravity on at once excites every
// mode, and BDF2's error on modes dt leaves unresolved converges at a lower
// rate: with a step load the bunny's ratio is 2.84 at 1/60, 2.90 at 1/120 and
// 3.22 at 1/240, still climbing toward 4.
TEST_P(Integrators, Bdf2IsSecondOrder) {
  constexpr Real dt = 1.0 / 60;
  constexpr Real end = 0.5;
  const Real bdf2 = ConvergenceRatio(
      GetParam(), Bdf2{.maxNewtonIterations = kIterations, .tolerance = kTight},
      dt, end);
  const Real backward = ConvergenceRatio(
      GetParam(),
      BackwardEuler{.maxNewtonIterations = kIterations, .tolerance = kTight},
      dt, end);
  std::cout << "[ measured ] " << GetParam() << " halving dt from 1/60 cuts "
            << "the error by " << bdf2 << " under Bdf2 and " << backward
            << " under BackwardEuler\n";
  EXPECT_GE(bdf2, 3);
  EXPECT_LE(bdf2, 5);
  EXPECT_GE(backward, 1.5);
  EXPECT_LE(backward, 2.5);
}

// One Quasistatic step reaches static equilibrium, and BackwardEuler run
// until it is at rest lands on the same positions. A large dt damps the
// lowest mode in a few hundred steps rather than tens of thousands.
TEST_P(Integrators, QuasistaticMatchesBackwardEulerAtRest) {
  Body statics = Hanging(GetParam());
  const StepReport report = statics.Step(Quasistatic{}, 0);
  EXPECT_TRUE(report.converged);
  EXPECT_LT(statics.StaticResidual(), 1e-6);

  Body dynamics = Hanging(GetParam());
  const BackwardEuler method{.maxNewtonIterations = kIterations,
                             .tolerance = kTight};
  int step = 0;
  while (step < 20'000 && (step == 0 || dynamics.StaticResidual() > 1e-9 ||
                           dynamics.state.v.lpNorm<Eigen::Infinity>() > 1e-9)) {
    ASSERT_TRUE(dynamics.Step(method, 0.25).converged) << "step " << step;
    ++step;
  }
  const Real displacement =
      (statics.state.x - statics.mesh->Positions()).lpNorm<Eigen::Infinity>();
  const Real gap =
      (statics.state.x - dynamics.state.x).lpNorm<Eigen::Infinity>();
  std::cout << "[ measured ] " << GetParam() << " Quasistatic: "
            << report.newtonIterations << " Newton iterations, residual "
            << statics.StaticResidual() << "; BackwardEuler at rest after "
            << step << " steps; largest gap " << gap << " of displacement "
            << displacement << "\n";
  EXPECT_GT(displacement, 1e-4);
  EXPECT_LT(gap, 1e-4 * displacement);
}

// A Newton step cut off before it converges says so, and leaves a finite
// state.
TEST_P(Integrators, UnconvergedStepReportsFailure) {
  Body body = Hanging(GetParam());
  const StepReport report = body.Step(
      BackwardEuler{.maxNewtonIterations = 1, .tolerance = 0}, 1.0 / 20);
  EXPECT_FALSE(report.converged);
  EXPECT_EQ(report.newtonIterations, 1);
  EXPECT_TRUE(body.state.x.allFinite());
  EXPECT_TRUE(body.state.v.allFinite());
}

INSTANTIATE_TEST_SUITE_P(TetAndBunny, Integrators,
                         testing::Values("tet", "bunny"),
                         [](const auto &info) { return info.param; });

static_assert(!std::is_default_constructible_v<Integrator>);
static_assert(std::is_constructible_v<Integrator, BackwardEuler>);

} // namespace
