#pragma once

#include <LibMath.h>

#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace sim {

// Index of a body in SceneDesc::bodies.
struct BodyId {
  int index;
};

struct Strand {
  Mat<Real> points;
  std::vector<int> pinned;
  Real leftTwistRate = 0;  // rad/s applied to the first edge's material frame
  Real rightTwistRate = 0; // rad/s applied to the last edge's material frame
  Real collisionStiffness = 10;
  Real collisionEpsilon = 0.2;
};

struct NeoHookean {
  Real youngsModulus;
  Real poissonRatio;
};
using Material = std::variant<NeoHookean>;

struct SoftBody {
  fs::path mesh;
  Material material;
  // Vertices strictly inside this box are pinned.
  Eigen::AlignedBox<Real, 3> pinRegion;
};

// Static triangle mesh strands collide against.
struct Collider {
  fs::path mesh;
  Vec3<Real> offset = Vec3<Real>::Zero();
};

using Body = std::variant<Strand, SoftBody, Collider>;

// An action is active for every step in [firstStep, lastStep] and runs before
// that step integrates. Steps count from 1. Paths sample Eigen::LinSpaced with
// one sample per `holdSteps` steps.
struct MovePinned {
  BodyId strand;
  int vertex;
  Real fromDx; // x offset from the vertex's rest position
  Real toDx;
};

struct MoveCollider {
  BodyId collider;
  Vec3<Real> from;
  Vec3<Real> to;
};

struct SetTwistRates {
  BodyId strand;
  Real left;
  Real right;
};

// Velocity damping coefficient used by the strand integrator.
struct SetDamping {
  BodyId strand;
  Real coefficient;
};

struct Action {
  int firstStep;
  std::optional<int> lastStep; // empty: stays active for the rest of the run
  int holdSteps = 1;
  std::variant<MovePinned, MoveCollider, SetTwistRates, SetDamping> effect;
};

// Viewer-only hints. The simulation never reads these.
struct ViewHints {
  std::optional<Vec3<float>> eye, lookAt, up, displacement;
  std::optional<float> zoom;
  bool showMaterialFrames = false;
};

struct SceneDesc {
  std::string name;
  Real dt;
  Vec3<Real> gravity{0, -0.981, 0};
  Real collisionEnvelope = 0.4;
  std::vector<Body> bodies;
  std::vector<Action> script;
  std::optional<int> lastStep; // empty: runs forever
  ViewHints view;
};

} // namespace sim
