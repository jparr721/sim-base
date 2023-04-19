#include "OpenGL.h"
#include <StrandMesh.h>

StrandMesh::StrandMesh(const Mat3<Real> &points) {
  // Fill modified bishop frames from the bag of points
  ASSERT(points.size() % 2 == 0, "The number of points must be even");

  for (int ii = 0; ii < points.rows() / 2; ii += 2) {
    rodSegments.emplace_back(points.row(ii), points.row(ii + 1));
  }
}

void StrandMesh::Draw() {
  glBegin(GL_LINES);
  for (const auto &rodSegment : rodSegments) {
    // Make a random color per segment.
    glColor3f(rand(), rand(), rand());
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv(rodSegment.x1.data());

    // Draw the bishop frame from t0, u, v
    glColor3f(1, 0, 0);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv((rodSegment.x0 + rodSegment.t0).eval().data());
    glColor3f(0, 1, 0);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv((rodSegment.x0 + rodSegment.u).eval().data());
    glColor3f(0, 0, 1);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv((rodSegment.x0 + rodSegment.v).eval().data());
  }
  glEnd();
}

auto StrandMesh::CurvatureBinormal(const Vec3<Real> &e0, const Vec3<Real> &e1)
    -> Vec3<Real> {
  return 2 * e0.cross(e1) / (e0.dot(e1) + e0.norm() * e1.norm());
}

auto StrandMesh::MaterialCurvature(const Vec3<Real> &curvatureBinormal,
                                   const MaterialFrame &frame) -> Vec2<Real> {
  return {curvatureBinormal.dot(frame.m1), curvatureBinormal.dot(frame.m2)};
}
ModifiedBishopFrame::ModifiedBishopFrame(const Vec3<Real> &x0,
                                         const Vec3<Real> &x1)
    : x0(x0), x1(x1), theta0(0), theta1(0) {
  e0 = x1 - x0;
  t0 = e0.normalized();

  // Get the values of u and v
  const auto orthogonalVectors = GetOrthogonalVectors(t0);
  u = orthogonalVectors.first;
  v = orthogonalVectors.second;
}
