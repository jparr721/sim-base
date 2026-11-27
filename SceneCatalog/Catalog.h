#pragma once

#include <Sim/Scene.h>

#include <span>
#include <string_view>

namespace sim {

struct SceneEntry {
  std::string_view name;
  SceneDesc (*make)();
};

// Every built-in scene, in a fixed order.
auto Catalog() -> std::span<const SceneEntry>;

} // namespace sim
