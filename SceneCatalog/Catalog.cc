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

// The bunny falls onto a frictionless 30 degree ramp, slides down it and comes
// to rest on the floor, over 14 simulated seconds. Only drag can stop it on a
// frictionless floor.
auto BunnyRamp(std::string name, Real dt, const Integrator &integrator,
               const Damping &damping, Real collisionEpsilon) -> SceneDesc {
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
               .material =
                   StableNeoHookean{.lame = Lame::FromYoungs(5.0, 0.45)},
               .integrator = integrator,
               .damping = damping,
               .collisionEpsilon = collisionEpsilon,
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
  return BunnyRamp("bunny-drop", 1.0 / 1'000.0, ForwardEuler{}, NoDamping{},
                   0.02);
}

// The same scene at a 20 Hz game tick. Newton gets 25 iterations. Where
// clamping is active it converges linearly, and the worst step of this run
// takes 14.
auto BunnyDropImplicit() -> SceneDesc {
  return BunnyRamp("bunny-drop-implicit", 1.0 / 20,
                   BackwardEuler{.maxNewtonIterations = 25}, NoDamping{}, 0.02);
}

// bunny-drop-implicit with Green-strain damping. The undamped bunny keeps
// jiggling as it slides, and its max vertex speed stays above 1e-3 until
// step 370 (18.5 s). This one stays below it from step 275 (13.75 s), and
// Newton's worst step takes 7 iterations instead of 14.
auto BunnyDropImplicitDamped() -> SceneDesc {
  return BunnyRamp("bunny-drop-implicit-damped", 1.0 / 20,
                   BackwardEuler{.maxNewtonIterations = 25},
                   GreenDamping{.mu = 2}, 0.02);
}

auto StrandBend() -> SceneDesc {
  return {
      .name = "strand-bend",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(5),
                        .place = {.centroid = Vec3<Real>(2, 0, 0)},
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

// The comb sweeps one sample per 50 steps once step 20000 has passed, and
// damping drops to 0.45 one hold later.
auto StrandDrop() -> SceneDesc {
  SceneDesc desc{
      .name = "strand-drop",
      .dt = 1.0 / 1'000.0,
      .lastStep = 30'000,
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
                                    .pinned = {0},
                                    .collisionStiffness = 5,
                                    .collisionEpsilon = 0.3});
    desc.script.push_back(
        {.firstStep = 20'051,
         .effect = SetDamping{.strand = strand, .coefficient = 0.45}});
  }
  const BodyId comb{static_cast<int>(desc.bodies.size())};
  desc.bodies.emplace_back(Collider{
      .shape = MeshFile{Meshes / "comb_more_teeth.obj"},
      .place = {.centroid = Vec3<Real>(-1.0181996320738793,
                                       -0.99999922397060237,
                                       9.9997512330105884)}});
  desc.script.push_back({.firstStep = 20'001,
                         .lastStep = 30'000,
                         .holdSteps = 50,
                         .effect = MoveCollider{.collider = comb,
                                                .from = Vec3<Real>(0, 0, 0),
                                                .to = Vec3<Real>(0, -7, 0)}});
  return desc;
}

auto FrameDebug() -> SceneDesc {
  return {
      .name = "frame-debug",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(5),
                        .place = {.centroid = Vec3<Real>(2, 1, 0)}}},
      .lastStep = 0,
      .view = {.zoom = kZoom, .showMaterialFrames = true},
  };
}

constexpr std::array kCatalog = {
    SceneEntry{"bunny-drop", BunnyDrop},
    SceneEntry{"bunny-drop-implicit", BunnyDropImplicit},
    SceneEntry{"bunny-drop-implicit-damped", BunnyDropImplicitDamped},
    SceneEntry{"strand-bend", StrandBend},
    SceneEntry{"strand-twist", StrandTwist},
    SceneEntry{"strand-drop", StrandDrop},
    SceneEntry{"frame-debug", FrameDebug},
};

} // namespace

auto Catalog() -> std::span<const SceneEntry> { return kCatalog; }

} // namespace sim
