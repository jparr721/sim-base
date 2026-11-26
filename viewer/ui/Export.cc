#include <ui/Panels.h>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace viewer::ui {

void Export(ViewerState &state) {
  auto &exporter = state.exporter;
  ImGui::InputText("directory", &exporter.directory);
  ImGui::BeginDisabled(exporter.directory.empty() || !state.world);
  if (ImGui::Button("Export frame")) {
    ExportCurrentFrame(state);
  }
  ImGui::SameLine();
  ImGui::Checkbox("while playing", &exporter.everyFrame);
  ImGui::EndDisabled();
  if (!exporter.status.empty()) {
    ImGui::TextWrapped("%s", exporter.status.c_str());
  }
}

} // namespace viewer::ui
