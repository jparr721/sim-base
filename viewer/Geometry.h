#pragma once

#include <array>
#include <vector>

namespace sim {
class World;
}

namespace viewer {

struct ColorVertex {
  std::array<float, 3> position;
  std::array<float, 3> color;
  float pointSize;
};

struct LitVertex {
  std::array<float, 3> position;
  std::array<float, 3> normal;
  std::array<float, 3> color;
};

// One frame's draw lists, rebuilt from the World every frame. Holds no GL
// state, so it is the seam between the simulation and the renderer.
struct FrameGeometry {
  std::vector<ColorVertex> lines;  // GL_LINES pairs
  std::vector<ColorVertex> points; // GL_POINTS
  std::vector<LitVertex> triangles;

  void Clear();
};

void AppendWorld(FrameGeometry &geometry, const sim::World &world);
void AppendGridAndAxes(FrameGeometry &geometry);

} // namespace viewer
