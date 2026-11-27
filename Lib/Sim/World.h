#pragma once

#include <Collision/CollisionMesh.h>
#include <Sim/Scene.h>
#include <StrandMesh.h>
#include <TetMesh.h>

#include <memory>
#include <span>
#include <variant>
#include <vector>

namespace sim {

// Read-only view of one body's live state, in SceneDesc::bodies order.
using BodyState =
    std::variant<const StrandMesh *, const TetMesh *, const CollisionMesh *>;

// Owns every mesh and integrator a SceneDesc describes. Construction is the
// only place a description turns into simulation objects. A malformed
// description throws std::invalid_argument.
class World {
public:
  explicit World(SceneDesc desc);
  ~World();
  World(World &&) noexcept;
  auto operator=(World &&) noexcept -> World &;

  // Runs the actions active for the next step, detects collisions, then
  // integrates every body. Does nothing once Finished().
  void Step();

  // Added to the scene's gravity on subsequent steps (interactive pulls).
  void SetExtraAcceleration(const Vec3<Real> &acceleration);

  [[nodiscard]] auto Steps() const -> int;
  [[nodiscard]] auto Finished() const -> bool;
  [[nodiscard]] auto Desc() const -> const SceneDesc &;
  [[nodiscard]] auto Bodies() const -> std::span<const BodyState>;
  // Vertex-face contacts found on the last step, across all strands.
  [[nodiscard]] auto Contacts() const -> std::span<const VertexFaceCollision>;

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};

} // namespace sim
