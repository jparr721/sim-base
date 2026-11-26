#pragma once

#include <Geometry.h>

#include <Eigen/Core>

namespace viewer {

// Two shader programs: unlit colored lines and points, and lit triangles.
// Each Draw refills dynamic vertex buffers from the frame's geometry.
// Requires a current OpenGL 4.1 core context for its whole lifetime.
class Renderer {
public:
  Renderer();
  ~Renderer();
  Renderer(const Renderer &) = delete;
  auto operator=(const Renderer &) -> Renderer & = delete;

  // Clears the framebuffer and draws the geometry over it.
  void Draw(const FrameGeometry &geometry, const Eigen::Matrix4f &view,
            const Eigen::Matrix4f &projection, int width, int height);

private:
  struct Stream {
    unsigned int vao = 0;
    unsigned int vbo = 0;
  };

  unsigned int colorProgram = 0;
  unsigned int litProgram = 0;
  Stream lines;
  Stream points;
  Stream triangles;
};

} // namespace viewer
