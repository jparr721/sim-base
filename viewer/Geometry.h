#pragma once

#include <RenderSettings.h>

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

// A soft-body surface vertex. The rest position and rest normal pin the
// texture to the material while the body deforms.
struct SurfaceVertex {
  std::array<float, 3> position;
  std::array<float, 3> normal;
  std::array<float, 3> restPosition;
  std::array<float, 3> restNormal;
};

// One frame's draw lists, rebuilt from the World every frame. Holds no GL
// state, so it is the seam between the simulation and the renderer.
struct FrameGeometry {
  std::vector<ColorVertex> lines;  // GL_LINES pairs
  std::vector<ColorVertex> points; // GL_POINTS
  std::vector<LitVertex> triangles; // collider surfaces, flat shaded
  std::vector<SurfaceVertex> surfaceVertices; // soft bodies, smooth shaded
  std::vector<unsigned int> surfaceIndices;   // triangles into surfaceVertices

  void Clear();
};

void AppendWorld(FrameGeometry &geometry, const sim::World &world,
                 const RenderSettings &settings);
void AppendGridAndAxes(FrameGeometry &geometry);

} // namespace viewer
