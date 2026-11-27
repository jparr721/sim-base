#include <ui/Panels.h>

#include <imgui.h>

#include <array>

namespace viewer::ui {

auto Panels() -> std::span<const Panel> {
  static constexpr std::array kPanels{
      Panel{"Scene", ScenePicker},
      Panel{"Playback", Playback},
      Panel{"Stats", Stats},
      Panel{"Lighting", Lighting},
      Panel{"Export", Export},
  };
  return kPanels;
}

void DrawPanels(ViewerState &state) {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::Begin("sim-base", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  for (const auto &panel : Panels()) {
    if (ImGui::CollapsingHeader(panel.title, ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID(panel.title);
      panel.draw(state);
      ImGui::PopID();
    }
  }
  ImGui::End();
}

} // namespace viewer::ui
