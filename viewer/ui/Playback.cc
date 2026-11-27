#include <ui/Panels.h>

#include <imgui.h>

namespace viewer::ui {

void Playback(ViewerState &state) {
  auto &playback = state.playback;
  if (ImGui::Button(playback.playing ? "Pause" : "Play")) {
    playback.playing = !playback.playing;
  }
  ImGui::SameLine();
  if (ImGui::Button("Step")) {
    playback.playing = false;
    playback.stepOnce = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    playback.playing = false;
    ResetWorld(state);
  }
  ImGui::SliderInt("steps / frame", &playback.stepsPerFrame, 1, 500);
  ImGui::Checkbox("grid and axes", &state.showGrid);
  ImGui::TextDisabled("a play/pause, space step, g grid, arrows pull");
}

} // namespace viewer::ui
