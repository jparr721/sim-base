#pragma once

#include <array>

namespace viewer {

// How soft bodies are lit and drawn. Positions are world space; colors are
// linear RGB.
struct RenderSettings {
  std::array<float, 3> lightPosition{1.2f, 2.4f, 1.6f};
  std::array<float, 3> lightColor{1.0f, 0.95f, 0.88f};
  float lightIntensity = 6.0f;
  float ambientStrength = 0.25f;
  float textureScale = 2.5f; // texture repeats per world unit
  bool wireframe = false;
  bool pins = false;
};

} // namespace viewer
