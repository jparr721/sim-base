#include "Catalog.h"

#include <Settings.h>

#include <array>
#include <limits>

namespace sim {

namespace {

constexpr float kZoom = 10;

auto LineStrand(int nVertices, Real y) -> Mat<Real> {
  Mat<Real> points(nVertices, 3);
  for (int ii = 0; ii < nVertices; ++ii) {
    points.row(ii) = Vec3<Real>(ii, y, 0);
  }
  return points;
}

auto BunnyDrop() -> SceneDesc {
  constexpr Real inf = std::numeric_limits<Real>::infinity();
  return {
      .name = "bunny-drop",
      .dt = 1.0 / 3000.0,
      .bodies = {SoftBody{
          .mesh = Meshes / "bunny.obj",
          .material = NeoHookean{.youngsModulus = 5.0, .poissonRatio = 0.45},
          .pinRegion = {Vec3<Real>(-inf, 0.9, -inf), Vec3<Real>(inf, inf, inf)},
      }},
      .view = {.zoom = kZoom},
  };
}

auto StrandBend() -> SceneDesc {
  return {
      .name = "strand-bend",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(5, 0),
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
      .bodies = {Strand{.points = LineStrand(8, 0),
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
// damping drops to 0.45 one hold later, as the legacy frame counters did.
auto StrandDrop() -> SceneDesc {
  SceneDesc desc{
      .name = "strand-drop",
      .dt = 1.0 / 1'000.0,
      .lastStep = 30'000,
      .view = {.eye = Vec3<float>(12.4994, 4.6141, 10.0808),
               .lookAt = Vec3<float>(-0.367183, -0.0835798, 9.81318),
               .up = Vec3<float>(-0.342824, 0.939373, -0.00713152),
               .displacement = Vec3<float>(-0.367183, -0.0835798, 9.81318),
               .zoom = kZoom},
  };
  for (Real ss = 8; ss < 12; ss += 0.05) {
    std::vector<Vec3<Real>> points;
    for (Real ii = 0; ii < 8; ii += 0.25) {
      points.emplace_back(ii, 0, ss);
    }
    Mat<Real> v(points.size(), 3);
    for (int ii = 0; ii < std::ssize(points); ++ii) {
      v.row(ii) = points.at(ii);
    }
    const BodyId strand{static_cast<int>(desc.bodies.size())};
    desc.bodies.emplace_back(Strand{.points = v,
                                    .pinned = {0},
                                    .collisionStiffness = 5,
                                    .collisionEpsilon = 0.3});
    desc.script.push_back(
        {.firstStep = 20'051,
         .effect = SetDamping{.strand = strand, .coefficient = 0.45}});
  }
  const BodyId comb{static_cast<int>(desc.bodies.size())};
  desc.bodies.emplace_back(Collider{.mesh = Meshes / "comb_more_teeth.obj",
                                    .offset = Vec3<Real>(-1, -1, 10)});
  desc.script.push_back({.firstStep = 20'001,
                         .lastStep = 30'000,
                         .holdSteps = 50,
                         .effect = MoveCollider{.collider = comb,
                                                .from = Vec3<Real>(-1, -1, 10),
                                                .to = Vec3<Real>(-1, -8, 10)}});
  return desc;
}

auto FrameDebug() -> SceneDesc {
  return {
      .name = "frame-debug",
      .dt = 1.0 / 1'000.0,
      .bodies = {Strand{.points = LineStrand(5, 1)}},
      .lastStep = 0,
      .view = {.zoom = kZoom, .showMaterialFrames = true},
  };
}

constexpr std::array kCatalog = {
    SceneEntry{"bunny-drop", BunnyDrop},
    SceneEntry{"strand-bend", StrandBend},
    SceneEntry{"strand-twist", StrandTwist},
    SceneEntry{"strand-drop", StrandDrop},
    SceneEntry{"frame-debug", FrameDebug},
};

} // namespace

auto Catalog() -> std::span<const SceneEntry> { return kCatalog; }

} // namespace sim
