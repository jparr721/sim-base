#include <ui/Panels.h>

#include <imgui.h>

namespace viewer::ui {

void Lighting(ViewerState &state) {
  auto &render = state.render;
  ImGui::DragFloat3("light position", render.lightPosition.data(), 0.05f);
  ImGui::ColorEdit3("light color", render.lightColor.data());
  ImGui::SliderFloat("intensity", &render.lightIntensity, 0, 40);
  ImGui::SliderFloat("ambient", &render.ambientStrength, 0, 1);
  ImGui::SliderFloat("texture scale", &render.textureScale, 0.25f, 8,
                     "%.2f", ImGuiSliderFlags_Logarithmic);
  ImGui::Checkbox("wireframe", &render.wireframe);
  ImGui::SameLine();
  ImGui::Checkbox("pins", &render.pins);
}

} // namespace viewer::ui
