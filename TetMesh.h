#pragma once

#include "Math.h"
#include "OpenGL.h"
#include "Settings.h"
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>

class TetMesh {
public:
  Mat<Real> v;
  Mat<int> f;
  Mat<int> t;
  Mat<Real> n;

  explicit TetMesh(const fs::path &file) {
    igl::read_triangle_mesh(file.string(), v, f);
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
  }
  TetMesh(const Mat<Real> &v, const Mat<int> &f) : v(v), f(f) {
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
  }
  TetMesh(const Mat<Real> &v, const Mat<int> &f, const Mat<int> &t)
      : v(v), f(f), t(t) {
    igl::per_vertex_normals(v, f, n);
  }

  inline void Draw() {
    // Draw lines
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(-1.0, -1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3d(0.0, 0.0, 0.0);
    glBegin(GL_TRIANGLES);
    for (int ii = 0; ii < f.rows(); ++ii) {
      const Vec3<int> face = f.row(ii);
      const Vec3<Real> a = v.row(face(0));
      const Vec3<Real> b = v.row(face(1));
      const Vec3<Real> c = v.row(face(2));
      glVertex3dv(a.data());
      glVertex3dv(b.data());
      glVertex3dv(c.data());
    }
    glEnd();

    // Draw without lines
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4d(0.7, 0.7, 0.7, 0.25);
    glBegin(GL_TRIANGLES);
    for (int ii = 0; ii < f.rows(); ++ii) {
      const Vec3<int> face = f.row(ii);
      const Vec3<Real> a = v.row(face(0));
      const Vec3<Real> b = v.row(face(1));
      const Vec3<Real> c = v.row(face(2));
      glVertex3dv(a.data());
      glVertex3dv(b.data());
      glVertex3dv(c.data());
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_LINE);

    // Draw normals
    glColor3d(1.0, 0.0, 1.0);
    glBegin(GL_LINES);
    for (int ii = 0; ii < f.rows(); ++ii) {
      const Vec3<int> face = f.row(ii);
      const Vec3<Real> a = v.row(face(0));
      const Vec3<Real> b = v.row(face(1));
      const Vec3<Real> c = v.row(face(2));

      const Vec3<Real> an = n.row(face(0));
      const Vec3<Real> bn = n.row(face(1));
      const Vec3<Real> cn = n.row(face(2));

      glVertex3dv(a.data());
      glVertex3dv((a + an).eval().data());

      glVertex3dv(b.data());
      glVertex3dv((b + bn).eval().data());

      glVertex3dv(c.data());
      glVertex3dv((c + cn).eval().data());
    }
    glEnd();
  }

private:
  void Tetrahedralize();
};
