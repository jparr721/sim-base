#include "OpenGL.h"
#include <LibGui.h>
#include <StrandMesh.h>

StrandMesh::StrandMesh(const Mat<Real> &points)
    : der(new DiscreteElasticRod(points)),
      pinned(Vec<int>::Zero(points.rows())) {}

void StrandMesh::Draw() {
  if (drawMaterialFrame) {
    glBegin(GL_LINES);
    glLineWidth(10);
    for (int ii = 0; ii < der->nRods; ++ii) {
      const auto &bishopFrame = der->frames.at(ii);
      const Vec3<Real> x0 = der->vertices.row(ii);
      const Vec3<Real> x1 = der->vertices.row(ii + 1);

      // Draw the bishop frame from t0, u, v
      glColor3f(1, 0, 0);
      glVertex3dv(x0.data());
      glVertex3dv((x0 + bishopFrame.t).eval().data());

      glColor3f(0, 1, 0);
      glVertex3dv(x0.data());
      glVertex3dv((x0 + bishopFrame.u).eval().data());

      glColor3f(0, 0, 1);
      glVertex3dv(x0.data());
      glVertex3dv((x0 + bishopFrame.v).eval().data());
    }
    glEnd();
  }

  glBegin(GL_POINTS);
  for (int ii = 0; ii < der->nRods; ++ii) {
    const Vec3<Real> x0 = der->vertices.row(ii);
    const Vec3<Real> x1 = der->vertices.row(ii + 1);

    glColor3f(1, 1, 1);
    glVertex3dv(x0.data());
    glVertex3dv(x1.data());
  }
  glEnd();

  for (int ii = 0; ii < der->nRods; ++ii) {
    const Vec3<Real> x0 = der->vertices.row(ii);
    const Vec3<Real> x1 = der->vertices.row(ii + 1);

    glColor3f(1, 1, 1);
    DrawCylinder(x0, x1, 0.01);
  }
}

auto StrandMesh::ComputeMaterialForces(bool straight) -> Vec<Real> {
  if (straight) {
    return der->ComputeCenterlineForcesStraight();
  }

  Vec<Real> R = der->ComputeCenterlineForcesGeneral();

  // Add collision forces

  return R;
}
