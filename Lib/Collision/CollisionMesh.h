#pragma once

#include <LibMath.h>

namespace sim {

struct VertexFaceCollision {
  int vertexIndex;
  int faceIndex;
  std::vector<Vec3<Real>> faceVertices;
};

class CollisionMesh {
public:
  virtual ~CollisionMesh() = default;
  Mat<Real> v;
  Mat<Real> rv;
  Mat<int> f;

  virtual void Translate(const Vec3<Real> &translation);
  virtual auto DetectFaceCollisions(Real eps, const Mat<Real> &vertices)
      -> std::vector<VertexFaceCollision>;
};

} // namespace sim
