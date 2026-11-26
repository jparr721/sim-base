#include <ui/Panels.h>

#include <imgui.h>

#include <string>

namespace viewer::ui {

void ScenePicker(ViewerState &state) {
  const std::string current(state.catalog[state.sceneIndex].name);
  if (!ImGui::BeginCombo("scene", current.c_str())) {
    return;
  }
  for (std::size_t ii = 0; ii < state.catalog.size(); ++ii) {
    const std::string name(state.catalog[ii].name);
    const bool selected = ii == state.sceneIndex;
    if (ImGui::Selectable(name.c_str(), selected) && !selected) {
      LoadScene(state, ii);
    }
  }
  ImGui::EndCombo();
}

} // namespace viewer::ui
