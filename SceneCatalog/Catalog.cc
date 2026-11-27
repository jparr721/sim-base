#include "Catalog.h"

#include <Settings.h>

#include <array>
#include <cmath>
#include <numbers>
#include <string>

namespace sim {

namespace {

constexpr float kZoom = 10;

// Unit-spaced vertices along x, centered on the origin.
auto LineStrand(int nVertices) -> Mat<Real> {
  Mat<Real> points(nVertices, 3);
  for (int ii = 0; ii < nVertices; ++ii) {
    points.row(ii) = Vec3<Real>(ii - (nVertices - 1) / 2.0, 0, 0);
  }
  return points;
}

// The velocity filter v <- (I - dt c M^-1) v that the explicit strand
// integrator applied after each step, as Rayleigh mass damping. A strand
// vertex has mass m = 0.2, so the filter is an explicit step of the force
// -c v = -(c / m) M v.
auto StrandDamping(Real c) -> Damping { return Rayleigh{.mass = c / 0.2}; }

// The bunny's material stiffness, E = 5 and nu = 0.45.
const Lame kBunnyLame = Lame::FromYoungs(5.0, 0.45);

// The bunny falls onto a frictionless 30 degree ramp, slides down it and comes
// to rest on the floor, over 14 simulated seconds. Only drag can stop it on a
// frictionless floor.
auto BunnyRamp(std::string name, Real dt, const Integrator &integrator,
               const Damping &damping) -> SceneDesc {
  constexpr Real theta = std::numbers::pi / 6;
  const Vec3<Real> downhill(std::cos(theta), -std::sin(theta), 0);
  const Vec3<Real> rampNormal(std::sin(theta), std::cos(theta), 0);
  const Vec3<Real> rampHalf(1.5, 0.1, 1);
  // The center of the ramp's top face, which meets the floor 1.2 downhill of
  // here. The last 0.3 of the ramp is under the floor.
  const Vec3<Real> rampTop(0, 1.2 * std::sin(theta), 0);
  const Vec3<Real> floorHalf(2.75, 0.25, 1.5);
  return {
      .name = std::move(name),
      .dt = dt,
      .bodies =
          {SoftBody{
               .mesh = {Meshes / "bunny.obj"},
               .place = {.centroid = rampTop - 0.8 * downhill +
                                     Vec3<Real>(0, 0.66, 0)},
               .material = StableNeoHookean{kBunnyLame},
               .integrator = integrator,
               .damping = damping,
               .collisionEpsilon = 0.02,
               .drag = 0.75,
           },
           Collider{.shape = Box{rampHalf},
                    .place = {.centroid = rampTop - rampHalf.y() * rampNormal,
                              .rotation = Eigen::Quaternion<Real>(
                                  Eigen::AngleAxis<Real>(
                                      -theta, Vec3<Real>::UnitZ()))}},
           Collider{.shape = Box{floorHalf},
                    .place = {.centroid = Vec3<Real>(1.25, -floorHalf.y(), 0)}}},
      .lastStep = static_cast<int>(std::lround(14 / dt)),
      // From the front and a little above, centered on the ramp's foot. The
      // camera orbits `displacement`, and `eye` is its offset from there.
      .view = {.eye = Vec3<float>(0.8f, 1.6f, 4.6f),
               .displacement = Vec3<float>(1.25f, 0.8f, 0.0f)},
  };
}

auto BunnyDrop() -> SceneDesc {
  return BunnyRamp("bunny-drop", 1.0 / 1'000.0, ForwardEuler{}, NoDamping{});
}

// The same scene at a 20 Hz game tick. Newton gets 25 iterations. Where
// clamping is active it converges linearly, and the worst step of this run
// takes 14.
auto BunnyDropImplicit() -> SceneDesc {
  return BunnyRamp("bunny-drop-implicit", 1.0 / 20,
                   BackwardEuler{.maxNewtonIterations = 25}, NoDamping{});
}

// bunny-drop-implicit with Green-strain damping. The undamped bunny keeps
// jiggling as it slides, and its max vertex speed stays above 1e-3 until
// step 370 (18.5 s). This one stays below it from step 275 (13.75 s), and
// Newton's worst step takes 7 iterations instead of 14.
auto BunnyDropImplicitDamped() -> SceneDesc {
  return BunnyRamp("bunny-drop-implicit-damped", 1.0 / 20,
                   BackwardEuler{.maxNewtonIterations = 25},
                   GreenDamping{.mu = 2});
}

// The catalog's energies, each demonstrated on bunny-drop-implicit-damped
// with its material swapped. Every isotropic energy uses the bunny's Lame
// parameters.
auto BunnyEnergy(std::string name, Material material,
                 std::optional<FiberMaterial> fiber = {}) -> SceneDesc {
  SceneDesc desc = BunnyDropImplicitDamped();
  desc.name = std::move(name);
  auto &bunny = std::get<SoftBody>(desc.bodies[0]);
  bunny.material = std::move(material);
  bunny.fiber = std::move(fiber);
  return desc;
}

// StVK's energy is zero at every reflection, so an inverted tet can rest
// inverted. From step 38, up to 37 tets invert on impact and stay inverted
// near J = -1, and the bunny still slides to rest past the ramp's foot.
// Newton takes 12 iterations at worst.
auto BunnyStVenantKirchhoff() -> SceneDesc {
  return BunnyEnergy("bunny-stvk", StVenantKirchhoff{kBunnyLame});
}

// The reference energy. It is not rotation invariant: turning a tet strains
// it, so the bunny resists turning as if it were stretched. It turns at most
// 11 degrees from rest on the way down, where the SNH bunny turns 26, and it
// still comes to rest past the ramp's foot.
auto BunnyLinearElasticity() -> SceneDesc {
  return BunnyEnergy("bunny-linear-elasticity", LinearElasticity{kBunnyLame});
}

auto BunnyNeoHookean() -> SceneDesc {
  return BunnyEnergy("bunny-neo-hookean", NeoHookeanBonetWood{kBunnyLame});
}

// ARAP has no volume term, so nothing resists a sliver tet flipping. Two tets
// invert near step 119 and recover. Under the base bunny's Green damping
// (mu = 2) the bunny still moves 5.2e-3 over the last second, and with
// mu = 2.25 it settles to 1.7e-3. Settling is not monotone in mu: 2.5 gives
// 0.9e-3 and 3.5 gives 2.2e-3.
auto BunnyArap() -> SceneDesc {
  SceneDesc desc = BunnyEnergy("bunny-arap", Arap{kBunnyLame});
  std::get<SoftBody>(desc.bodies[0]).damping = GreenDamping{.mu = 2.25};
  return desc;
}

auto BunnySymmetricDirichlet() -> SceneDesc {
  return BunnyEnergy("bunny-symmetric-dirichlet",
                     SymmetricDirichlet{kBunnyLame});
}

// The barrier acts only where J < 0.25. No tet comes below J = 0.55 on this
// ramp, so the run matches bunny-drop-implicit-damped to 2e-11. Where the
// barrier is not needed, it changes nothing.
auto BunnySnhBarrier() -> SceneDesc {
  return BunnyEnergy("bunny-snh-barrier", StableNeoHookeanBarrier{kBunnyLame});
}

// The SNH bunny with a fiber along the ramp's slope. The bunny is placed
// unrotated, so its local axes are the world's. The fiber's E = 20 is four
// times the bunny's. The bunny barely stretches along the slope, so each
// fiber moves its vertices at most 0.023 from bunny-drop-implicit-damped's.
const Vec3<Real> kSlopeFiber =
    Vec3<Real>(std::cos(std::numbers::pi / 6), -std::sin(std::numbers::pi / 6),
               0);
const Lame kFiberLame = Lame::FromYoungs(20.0, 0.45);

auto BunnyFiber(std::string name, FiberMaterial fiber) -> SceneDesc {
  return BunnyEnergy(std::move(name), StableNeoHookean{kBunnyLame},
                     std::move(fiber));
}

auto BunnyFiberStVenantKirchhoff() -> SceneDesc {
  return BunnyFiber("bunny-fiber-stvk",
                    AnisotropicStVenantKirchhoff{.lame = kFiberLame,
                                                 .fiber = kSlopeFiber});
}

auto BunnyFiberArap() -> SceneDesc {
  return BunnyFiber("bunny-fiber-arap",
                    AnisotropicArap{.lame = kFiberLame, .fiber = kSlopeFiber});
}

auto BunnyFiberFung() -> SceneDesc {
  return BunnyFiber("bunny-fiber-fung",
                    AnisotropicFung{.lame = kFiberLame, .fiber = kSlopeFiber});
}

auto StrandBend() -> SceneDesc {
  return {
      .name = "strand-bend",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(5),
                        .place = {.centroid = Vec3<Real>(2, 0, 0)},
                        .integrator = ForwardEuler{},
                        .damping = StrandDamping(0.75),
                        .pinned = {0, 4},
                        .collisionStiffness = 5,
                        .collisionEpsilon = 0.3}},
      .script = {{.firstStep = 1'002,
                  .lastStep = 6'001,
                  .effect = MovePinned{
                      .strand = {0}, .vertex = 0, .fromDx = 0, .toDx = 2.5}},
                 {.firstStep = 1'002,
                  .lastStep = 6'001,
                  .effect = MovePinned{
                      .strand = {0}, .vertex = 4, .fromDx = 0, .toDx = -2.5}}},
      .lastStep = 20'000,
      .view = {.zoom = kZoom},
  };
}

auto StrandTwist() -> SceneDesc {
  return {
      .name = "strand-twist",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(8),
                        .place = {.centroid = Vec3<Real>(3.5, 0, 0)},
                        .integrator = ForwardEuler{},
                        .damping = StrandDamping(0.75),
                        .pinned = {0, 7},
                        .rightTwistRate = 50,
                        .collisionStiffness = 5,
                        .collisionEpsilon = 0.3}},
      .script = {{.firstStep = 2'002,
                  .effect = SetTwistRates{.strand = {0}, .left = 0, .right = 0}}},
      .lastStep = 20'000,
      .view = {.zoom = kZoom},
  };
}

// Hair hangs from 80 pinned roots beside a comb, which sweeps down through it
// from 20 s to 30 s, one sample per 50 ms, and damping drops to 0.45 one
// sample later. The old velocity filter damped the velocity a step had just
// made, and Rayleigh damping acts on the velocity a step starts from, so the
// drop is set one step later than the filter's was.
auto StrandDrop(std::string name, Real dt, const Integrator &integrator)
    -> SceneDesc {
  const auto step = [&](Real seconds) {
    return static_cast<int>(std::lround(seconds / dt));
  };
  const int sweepStart = step(20) + 1;
  const int hold = step(0.05);
  SceneDesc desc{
      .name = std::move(name),
      .dt = dt,
      .lastStep = step(30),
      .view = {.eye = Vec3<float>(12.4994, 4.6141, 10.0808),
               .lookAt = Vec3<float>(-0.367183, -0.0835798, 9.81318),
               .up = Vec3<float>(-0.342824, 0.939373, -0.00713152),
               .displacement = Vec3<float>(-0.367183, -0.0835798, 9.81318)},
  };
  const Mat<Real> hair = 0.25 * LineStrand(32);
  for (Real ss = 8; ss < 12; ss += 0.05) {
    const BodyId strand{static_cast<int>(desc.bodies.size())};
    desc.bodies.emplace_back(Strand{.points = hair,
                                    .place = {.centroid = {3.875, 0, ss}},
                                    .integrator = integrator,
                                    .damping = StrandDamping(0.75),
                                    .pinned = {0},
                                    .collisionStiffness = 5,
                                    .collisionEpsilon = 0.3});
    desc.script.push_back(
        {.firstStep = sweepStart + hold + 1,
         .effect = SetDamping{.strand = strand,
                              .damping = StrandDamping(0.45)}});
  }
  const BodyId comb{static_cast<int>(desc.bodies.size())};
  desc.bodies.emplace_back(Collider{
      .shape = MeshFile{Meshes / "comb_more_teeth.obj"},
      .place = {.centroid = Vec3<Real>(-1.0181996320738793,
                                       -0.99999922397060237,
                                       9.9997512330105884)}});
  desc.script.push_back({.firstStep = sweepStart,
                         .lastStep = step(30),
                         .holdSteps = hold,
                         .effect = MoveCollider{.collider = comb,
                                                .from = Vec3<Real>(0, 0, 0),
                                                .to = Vec3<Real>(0, -7, 0)}});
  return desc;
}

auto StrandDropExplicit() -> SceneDesc {
  return StrandDrop("strand-drop", 1.0 / 1'000.0, ForwardEuler{});
}

// The same hair at a 60 Hz game tick. Over the first 600 steps Newton
// converges on every strand, in 2 iterations on average and 3 at worst.
auto StrandDropImplicit() -> SceneDesc {
  return StrandDrop("strand-drop-implicit", 1.0 / 60, BackwardEuler{});
}

auto FrameDebug() -> SceneDesc {
  return {
      .name = "frame-debug",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(5),
                        .place = {.centroid = Vec3<Real>(2, 1, 0)},
                        .integrator = ForwardEuler{},
                        .damping = StrandDamping(0.75)}},
      .lastStep = 0,
      .view = {.zoom = kZoom, .showMaterialFrames = true},
  };
}

constexpr std::array kCatalog = {
    SceneEntry{"bunny-drop", BunnyDrop},
    SceneEntry{"bunny-drop-implicit", BunnyDropImplicit},
    SceneEntry{"bunny-drop-implicit-damped", BunnyDropImplicitDamped},
    SceneEntry{"bunny-stvk", BunnyStVenantKirchhoff},
    SceneEntry{"bunny-linear-elasticity", BunnyLinearElasticity},
    SceneEntry{"bunny-neo-hookean", BunnyNeoHookean},
    SceneEntry{"bunny-arap", BunnyArap},
    SceneEntry{"bunny-symmetric-dirichlet", BunnySymmetricDirichlet},
    SceneEntry{"bunny-snh-barrier", BunnySnhBarrier},
    SceneEntry{"bunny-fiber-stvk", BunnyFiberStVenantKirchhoff},
    SceneEntry{"bunny-fiber-arap", BunnyFiberArap},
    SceneEntry{"bunny-fiber-fung", BunnyFiberFung},
    SceneEntry{"strand-bend", StrandBend},
    SceneEntry{"strand-twist", StrandTwist},
    SceneEntry{"strand-drop", StrandDropExplicit},
    SceneEntry{"strand-drop-implicit", StrandDropImplicit},
    SceneEntry{"frame-debug", FrameDebug},
};

} // namespace

auto Catalog() -> std::span<const SceneEntry> { return kCatalog; }

} // namespace sim
