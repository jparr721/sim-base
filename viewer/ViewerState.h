#pragma once

#include "Camera.h"
#include <SceneCatalog/Catalog.h>
#include <Sim/World.h>

#include <optional>
#include <span>
#include <string>

namespace viewer {

struct Playback {
  bool playing = false;
  bool stepOnce = false;  // advance exactly one step on the next frame
  int stepsPerFrame = 50; // the old drivers redrew every 50 steps
};

struct ExportSettings {
  std::string directory;
  bool everyFrame = false; // write one frame per rendered frame that stepped
  int nextFrame = 0;
  std::string status;
};

// Everything the frame loop and the UI panels share.
struct ViewerState {
  std::span<const sim::SceneEntry> catalog;
  std::size_t sceneIndex = 0;
  std::optional<sim::World> world; // empty when the scene failed to build
  std::string error;
  Camera<float> camera;
  int framebufferWidth = 1;
  int framebufferHeight = 1;
  Playback playback;
  ExportSettings exporter;
  Vec3<Real> pull = Vec3<Real>::Zero(); // extra acceleration for one step
  bool showGrid = true;
};

auto FindScene(std::span<const sim::SceneEntry> catalog, std::string_view name)
    -> std::optional<std::size_t>;

// Rebuilds the world from the current scene and leaves the camera alone.
void ResetWorld(ViewerState &state);

// Switches to catalog[index], rebuilds the world and frames the camera from
// the scene's view hints.
void LoadScene(ViewerState &state, std::size_t index);

// Runs this frame's playback budget. Returns the number of steps taken.
auto AdvanceWorld(ViewerState &state) -> int;

// Writes the world as the next numbered frame into exporter.directory and
// records the outcome in exporter.status.
void ExportCurrentFrame(ViewerState &state);

} // namespace viewer
