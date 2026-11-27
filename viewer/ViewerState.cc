#include <Export.h>
#include <ViewerState.h>

#include <algorithm>
#include <format>
#include <stdexcept>

namespace viewer {

auto FindScene(std::span<const sim::SceneEntry> catalog, std::string_view name)
    -> std::optional<std::size_t> {
  const auto it = std::ranges::find(catalog, name, &sim::SceneEntry::name);
  if (it == catalog.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(it - catalog.begin());
}

void ResetWorld(ViewerState &state) {
  state.world.reset();
  state.pull.setZero();
  state.playback.stepOnce = false;
  state.exporter.nextFrame = 0;
  try {
    state.world.emplace(state.catalog[state.sceneIndex].make());
    state.error.clear();
  } catch (const std::invalid_argument &e) {
    state.error = e.what();
  }
}

namespace {

void FrameCamera(ViewerState &state) {
  auto &camera = state.camera;
  camera = Camera<float>();
  camera.Resize(state.framebufferWidth, state.framebufferHeight);
  if (!state.world) {
    camera.Zoom(10);
    return;
  }
  const auto &hints = state.world->Desc().view;
  if (hints.eye) {
    camera.SetEye(*hints.eye);
  }
  if (hints.lookAt) {
    camera.SetLookAt(*hints.lookAt);
  }
  if (hints.up) {
    camera.SetUp(*hints.up);
  }
  if (hints.displacement) {
    camera.SetDisplacement(*hints.displacement);
  }
  if (hints.zoom) {
    camera.Zoom(*hints.zoom);
  }
}

} // namespace

void LoadScene(ViewerState &state, std::size_t index) {
  state.sceneIndex = index;
  state.playback.playing = false;
  ResetWorld(state);
  FrameCamera(state);
}

auto AdvanceWorld(ViewerState &state) -> int {
  if (!state.world) {
    return 0;
  }
  auto &playback = state.playback;
  const int budget =
      playback.playing ? playback.stepsPerFrame : (playback.stepOnce ? 1 : 0);
  playback.stepOnce = false;

  int taken = 0;
  for (; taken < budget && !state.world->Finished(); ++taken) {
    state.world->SetExtraAcceleration(state.pull);
    state.world->Step();
    state.pull.setZero();
  }
  return taken;
}

void ExportCurrentFrame(ViewerState &state) {
  auto &exporter = state.exporter;
  const auto written =
      ExportFrame(*state.world, exporter.directory, exporter.nextFrame);
  if (!written) {
    exporter.status = written.error();
    exporter.everyFrame = false;
    return;
  }
  exporter.status = std::format("frame {:04}: {} files in {}",
                                exporter.nextFrame, *written,
                                exporter.directory);
  ++exporter.nextFrame;
}

} // namespace viewer
