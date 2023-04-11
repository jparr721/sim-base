#pragma once

#include "LibMath.h"
#include "OpenGL.h"
#include "Settings.h"
#include <igl/boundary_facets.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>

constexpr std::array<float, 4> gAmbientFull = {{0.4, 0.4, 0.4, 1.0}};
constexpr std::array<float, 4> gSpecularFull = {1.0, 1.0, 1.0, 1.0};
constexpr std::array<float, 1> gShininess = {50.0};
constexpr std::array<float, 4> gDiffuseGray = {0.8, 0.8, 0.8, 1.0};

template <typename T> class TetMesh {
public:
  // Simulated vertices
  Mat<T> v;

  // Rest vertices
  Mat<T> rv;

  // Surface faces
  Mat<int> f;
  Mat<int> t;

  // Vertex normals
  Mat<T> n;

  // Per-element deformation gradients
  std::vector<Mat<T>> fs;

  explicit TetMesh(const fs::path &file) {
    igl::read_triangle_mesh(file.string(), v, f);
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
  }
  TetMesh(const Mat<T> &v, const Mat<int> &f) : v(v), f(f) {
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
  }
  TetMesh(const Mat<T> &v, const Mat<int> &f, const Mat<int> &t)
      : v(v), f(f), t(t) {
    igl::per_vertex_normals(v, f, n);
  }

  INLINE void Draw() {
    GLOBAL GLfloat lightAmbient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLOBAL GLfloat lightDiffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
    GLOBAL GLfloat lightSpecular[] = {0.5f, 0.5f, 0.5f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

    // USE THESE TO SHADE MATERIALS
    glShadeModel(GL_SMOOTH);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, gDiffuseGray.data());
    glMaterialfv(GL_FRONT, GL_AMBIENT, gAmbientFull.data());
    glMaterialfv(GL_FRONT, GL_SPECULAR, gSpecularFull.data());
    glMaterialfv(GL_FRONT, GL_SHININESS, gShininess.data());

    glEnable(GL_DEPTH_TEST);

    // Draw lines
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(-1.0, -1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3d(0.0, 0.0, 0.0);
    glBegin(GL_TRIANGLES);
    for (int ii = 0; ii < f.rows(); ++ii) {
      const Vec3<int> face = f.row(ii);
      const Vec3<T> a = v.row(face(0));
      const Vec3<T> b = v.row(face(1));
      const Vec3<T> c = v.row(face(2));
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
      const Vec3<T> a = v.row(face(0));
      const Vec3<T> b = v.row(face(1));
      const Vec3<T> c = v.row(face(2));
      glVertex3dv(a.data());
      glVertex3dv(b.data());
      glVertex3dv(c.data());
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_LINE);

    // Draw normals
    if (drawNormals) {
      glColor3d(1.0, 0.0, 1.0);
      glBegin(GL_LINES);
      for (int ii = 0; ii < f.rows(); ++ii) {
        const Vec3<int> face = f.row(ii);
        const Vec3<T> a = v.row(face(0));
        const Vec3<T> b = v.row(face(1));
        const Vec3<T> c = v.row(face(2));

        const Vec3<T> an = n.row(face(0));
        const Vec3<T> bn = n.row(face(1));
        const Vec3<T> cn = n.row(face(2));

        glVertex3dv(a.data());
        glVertex3dv((a + an).eval().data());

        glVertex3dv(b.data());
        glVertex3dv((b + bn).eval().data());

        glVertex3dv(c.data());
        glVertex3dv((c + cn).eval().data());
      }
      glEnd();
    }
  }
  INLINE void ToggleNormals() { drawNormals = !drawNormals; }
  INLINE auto FlatPositions() -> Vec<T> {
    Vec<T> fv(3 * v.rows());
    for (int ii = 0; ii < v.rows(); ++ii) {
      fv.segment<3>(3 * ii) = v.row(ii);
    }
    return fv;
  }
  INLINE static auto EvalPartialFPartialx(int index, const Mat3<Real> &dmInv) -> Mat3<T> {
    ASSERT(index >= 0 && index <= 11, "Index: " + std::to_string(index) +
                                          " invalid. Must be in range[0, 11].");

    Mat3<T> ret = Mat3<Real>::Zero();
    switch (index) {
    case 0: // pfpx0x
      ret.row(0) << -1, -1, -1;
      break;
    case 1: // pfpx0y
      ret.row(1) << -1, -1, -1;
      break;
    case 2: // pfpx0z
      ret.row(2) << -1, -1, -1;
      break;
    case 3: // pfpx1x
      ret(0, 0) = 1;
      break;
    case 4: // pfpx1y
      ret(1, 0) = 1;
      break;
    case 5: // pfpx1z
      ret(2, 0) = 1;
      break;
    case 6: // pfpx2x
      ret(0, 1) = 1;
      break;
    case 7: // pfpx2y
      ret(1, 1) = 1;
      break;
    case 8: // pfpx2z
      ret(2, 1) = 1;
      break;
    case 9: // pfpx3x
      ret(0, 2) = 1;
      break;
    case 10: // pfpx3y
      ret(1, 2) = 1;
      break;
    case 11: // pfpx3z
      ret(2, 2) = 1;
      break;
    }
    return ret * dmInv;
  }
  INLINE static auto PartialFPartialx(const Mat3<Real>& dmInv) -> Mat9x12<T> {
    Mat9x12<T> ret = Mat9x12<T>::Zero();
    for (int ii = 0; ii < 12; ++ii) {
      ret.col(ii) = Flatten(EvalPartialFPartialx(ii, dmInv));
    }
    return ret;
  }

private:
  bool drawNormals = false;

  // Per-element dm-inverses (for computing the deformation gradients)
  std::vector<Mat<T>> dmInvs;

  void InitializeDataStructures() { ComputeDeformationGradients(); }

  void Tetrahedralize() {
    static const std::string switches = "zpQ";

    igl::copyleft::tetgen::tetrahedralize(Mat<T>(v), Mat<int>(f), switches, v,
                                          t, f);
    igl::boundary_facets(t, f);

    // The faces come out of this function in the wrong winding order. So
    // this fixes that.
    f.rowwise().reverseInPlace();
  }
  void ComputeDmInverses() {
    for (int ii = 0; ii < t.rows(); ++ii) {
      const Vec4<int> tet = t.row(ii);
      const Vec3<T> a = rv.row(tet(0));
      const Vec3<T> b = rv.row(tet(1));
      const Vec3<T> c = rv.row(tet(2));
      const Vec3<T> d = rv.row(tet(3));
      Mat<T> dm(3, 3);
      dm.col(0) = b - a;
      dm.col(1) = c - a;
      dm.col(2) = d - a;
      dmInvs.emplace_back(dm.inverse());
    }
  }
  void ComputeDeformationGradients() {
    if (dmInvs.empty()) {
      ComputeDmInverses();
    }

    // Compute Ds values for each tet
    for (int ii = 0; ii < t.rows(); ++ii) {
      const Vec4<int> tet = t.row(ii);
      const Vec3<T> a = v.row(tet(0));
      const Vec3<T> b = v.row(tet(1));
      const Vec3<T> c = v.row(tet(2));
      const Vec3<T> d = v.row(tet(3));
      Mat<T> ds(3, 3);
      ds.col(0) = b - a;
      ds.col(1) = c - a;
      ds.col(2) = d - a;
      fs.emplace_back(ds * dmInvs[ii]);
    }
  }
};
