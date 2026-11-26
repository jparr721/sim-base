#include <ui/Panels.h>

#include <imgui.h>

namespace viewer::ui {

void Stats(ViewerState &state) {
  ImGui::Text("fps       %.1f", static_cast<double>(ImGui::GetIO().Framerate));
  if (!state.world) {
    ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "scene failed: %s",
                       state.error.c_str());
    return;
  }
  ImGui::Text("step      %d%s", state.world->Steps(),
              state.world->Finished() ? " (finished)" : "");
  ImGui::Text("contacts  %zu", state.world->Contacts().size());
  ImGui::Text("bodies    %zu", state.world->Bodies().size());
}

} // namespace viewer::ui
