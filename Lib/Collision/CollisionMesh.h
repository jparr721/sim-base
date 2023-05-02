#pragma once

#include <LibMath.h>

struct VertexFaceCollision {
  int vertexIndex;
  int faceIndex;
  std::vector<Vec3<Real>> faceVertices;
};

struct EdgeEdgeCollision {
  // Vertex indices of the edge-edge collision
  int e0;
  int e1;
};

class CollisionMesh {
public:
  Mat<Real> v;
  Mat<Real> rv;
  Mat<int> f;

  virtual void
  Draw(const std::vector<VertexFaceCollision> &vertexFaceCollisions);
  virtual void Translate(const Vec3<Real> &translation);
  virtual auto DetectFaceCollisions(Real eps, const Mat<Real> &vertices)
      -> std::vector<VertexFaceCollision>;
};
