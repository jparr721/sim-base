#include "CollisionMesh.h"
#include <LibGui.h>

void CollisionMesh::Draw(
    const std::vector<VertexFaceCollision> &vertexFaceCollisions) {
  DrawSurfaceMesh(v, f);

  if (!vertexFaceCollisions.empty()) {
    // Draw colored faces for each face in vertexFaceCollisions
    glBegin(GL_POINTS);
    for (const auto &collision : vertexFaceCollisions) {
      const Vec3<int> triangle = f.row(collision.faceIndex);
      const Vec3<Real> a = v.row(triangle(0));
      const Vec3<Real> b = v.row(triangle(1));
      const Vec3<Real> c = v.row(triangle(2));

      const Vec3<Real> color = Vec3<Real>(1, 0, 0);
      glColor3dv(color.data());
      glVertex3dv(a.data());
      glVertex3dv(b.data());
      glVertex3dv(c.data());
    }
    glEnd();
  }
}

void CollisionMesh::Translate(const Vec3<Real> &translation) {
  for (int ii = 0; ii < v.rows(); ++ii) {
    const Vec3<Real> restRow = rv.row(ii);
    v.row(ii) = restRow + translation;
  }
}

auto CollisionMesh::DetectFaceCollisions(Real eps, const Mat<Real> &vertices)
    -> std::vector<VertexFaceCollision> {
  std::vector<VertexFaceCollision> vertexFaceCollisions;
  for (int ii = 0; ii < vertices.rows(); ++ii) {
    const Vec3<Real> &vertex = vertices.row(ii);
    std::vector<std::pair<int, Real>> collisionFaceDistances;

    // Find the closest triangles in the comb
    for (int jj = 0; jj < f.rows(); ++jj) {
      const Vec3<int> triangle = f.row(jj);
      const Vec3<Real> a = v.row(triangle(0));
      const Vec3<Real> b = v.row(triangle(1));
      const Vec3<Real> c = v.row(triangle(2));

      // Find the closest point on the triangle to the vertex
      const Real distance = PointTriangleDistance(a, b, c, vertex);

      if (distance < eps) {
        collisionFaceDistances.emplace_back(jj, distance);
      }
    }

    // If nothing is colliding, move on.
    if (collisionFaceDistances.empty()) {
      continue;
    }

    for (const auto &collision : collisionFaceDistances) {
      VertexFaceCollision vertexFaceCollision = {};
      vertexFaceCollision.faceIndex = collision.first;
      vertexFaceCollision.vertexIndex = ii;
      vertexFaceCollision.faceVertices = {
          v.row(f(vertexFaceCollision.faceIndex, 0)),
          v.row(f(vertexFaceCollision.faceIndex, 1)),
          v.row(f(vertexFaceCollision.faceIndex, 2)),
      };
      vertexFaceCollisions.emplace_back(vertexFaceCollision);
    }
  }
  return vertexFaceCollisions;
}
