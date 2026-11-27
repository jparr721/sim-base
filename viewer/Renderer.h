#pragma once

#include <Geometry.h>
#include <RenderSettings.h>

#include <Eigen/Core>

namespace viewer {

// Shader programs for a background gradient, unlit colored lines and points,
// flat-lit collider triangles and textured, point-lit soft-body surfaces.
// Each Draw refills dynamic vertex buffers from the frame's geometry.
// Requires a current OpenGL 4.1 core context for its whole lifetime.
class Renderer {
public:
  Renderer();
  ~Renderer();
  Renderer(const Renderer &) = delete;
  auto operator=(const Renderer &) -> Renderer & = delete;

  // Clears the framebuffer and draws the geometry over it.
  void Draw(const FrameGeometry &geometry, const RenderSettings &settings,
            const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection,
            int width, int height);

private:
  struct Stream {
    unsigned int vao = 0;
    unsigned int vbo = 0;
  };

  void DrawSurfaces(const FrameGeometry &geometry,
                    const RenderSettings &settings,
                    const Eigen::Matrix4f &view);

  unsigned int backgroundProgram = 0;
  unsigned int colorProgram = 0;
  unsigned int litProgram = 0;
  unsigned int surfaceProgram = 0;
  unsigned int albedoTexture = 0;
  unsigned int backgroundVao = 0; // attribute-less fullscreen triangle
  unsigned int surfaceIndexBuffer = 0;
  Stream lines;
  Stream points;
  Stream triangles;
  Stream surfaces;
};

} // namespace viewer
