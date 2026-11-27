#pragma once

#include <Energy/Damping/Damping.h>
#include <Energy/Volume/FiberMaterial.h>
#include <Energy/Volume/Material.h>
#include <LibMath.h>
#include <Solid/Integrators.h>

#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace sim {

// Index of a body in SceneDesc::bodies.
struct BodyId {
  int index;
};

// Where a body sits in the world. World recenters the body's rest shape on
// its centroid, then scales, rotates and moves it so the centroid lands here.
struct Placement {
  Vec3<Real> centroid = Vec3<Real>::Zero();
  Eigen::Quaternion<Real> rotation = Eigen::Quaternion<Real>::Identity();
  Real scale = 1;
};

struct MeshFile {
  fs::path path;
};

// A closed box centered on its body's centroid.
struct Box {
  Vec3<Real> halfExtents;
};

// Points are body-local. Their vertex mean is the centroid.
struct Strand {
  Mat<Real> points;
  Placement place;
  // Required: Integrator has no default, so leaving this out does not compile.
  Integrator integrator;
  // Required, the same way: NoDamping{} or Rayleigh{.mass = ..., .stiffness =
  // ...}. GreenDamping needs a deformation gradient, which a strand lacks.
  Damping damping;
  std::vector<int> pinned;
  Real leftTwistRate = 0;  // rad/s applied to the first edge's material frame
  Real rightTwistRate = 0; // rad/s applied to the last edge's material frame
  Real collisionStiffness = 10;
  Real collisionEpsilon = 0.2;
};

// The tetrahedralized mesh is recentered on its volume-weighted centroid.
// Its surface vertices collide with every Collider, frictionlessly.
struct SoftBody {
  MeshFile mesh;
  Placement place;
  Material material;
  // Added to `material` in every tet. Its fiber direction is uniform,
  // body-local and unit length, and Placement's rotation turns it with the
  // body.
  std::optional<FiberMaterial> fiber;
  // Required: Integrator has no default, so leaving this out does not compile.
  Integrator integrator;
  // Required, the same way: NoDamping{}, GreenDamping{.mu = ...} or
  // Rayleigh{.mass = ..., .stiffness = ...}.
  Damping damping;
  // Vertices strictly inside this body-local box (recentered, before scale
  // and rotation) are pinned.
  std::optional<Eigen::AlignedBox<Real, 3>> pinRegion;
  // Per unit vertex mass: 1/s^2 for stiffness and 1/s for damping. A vertex
  // within collisionEpsilon of a collider triangle is pushed back out to
  // collisionEpsilon, and its velocity along the triangle normal is damped.
  Real collisionStiffness = 1e4;
  Real collisionEpsilon = 0.02;
  Real collisionDamping = 50;
  // 1/s: mass-proportional damping, -drag M v.
  Real drag = 0;
};

// Static triangle mesh that strands and soft bodies collide against,
// recentered on its area-weighted centroid.
struct Collider {
  std::variant<MeshFile, Box> shape;
  Placement place;
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

// Moves the collider's centroid along a path of offsets from place.centroid.
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

// Replaces the strand's damping, starting with this step's forces.
struct SetDamping {
  BodyId strand;
  Damping damping;
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
