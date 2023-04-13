#pragma once

#include "HyperelasticMaterial.h"
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

  // Mass Matrix
  SparseMat<T> m;
  SparseMat<T> mInv;

  // Rayleigh Damping
  SparseMat<T> rayleighDamping;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<T> pinned;

  // Per-element deformation gradients
  std::vector<Mat<T>> fs;

  explicit TetMesh(const fs::path &file) {
    igl::read_triangle_mesh(file.string(), v, f);
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
    InitializeDataStructures();
  }
  TetMesh(const Mat<T> &v, const Mat<int> &f) : v(v), f(f) {
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
    InitializeDataStructures();
  }
  TetMesh(const Mat<T> &v, const Mat<int> &f, const Mat<int> &t)
      : v(v), f(f), t(t) {
    igl::per_vertex_normals(v, f, n);
    InitializeDataStructures();
  }

  INLINE void ComputeDeformationGradients() {
    ASSERT2(!dmInvs.empty());
    ASSERT2(!partialFPartialxs.empty());

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
      fs.at(ii) = ds * dmInvs[ii];
    }
  }

  INLINE auto
  ComputeMaterialForces(const std::shared_ptr<HyperelasticMaterial> &material)
      -> Vec<T> {
    std::vector<Vec12<T>> perElementForces(t.rows());
    for (int tt = 0; tt < t.rows(); ++tt) {
      const Mat3<T> &F = fs[tt];
      const Mat3<T> &P = material->Pk1(F);
      const Vec12<T> forceDensity =
          partialFPartialxs.at(tt).transpose() * Flatten(P);
      const Vec12<T> force = -volumes.at(tt) * forceDensity;
      perElementForces.at(tt) = force;
    }

    // Scatter global forces
    Vec<T> forces = Vec<T>::Zero(DOFs());
    for (int tt = 0; tt < t.rows(); ++tt) {
      const Vec4<int> tet = t.row(tt);
      const Vec12<T> force = perElementForces.at(tt);

      for (int ii = 0; ii < 4; ++ii) {
        int index = 3 * tet(ii);
        forces(index) += force(3 * ii);
        forces(index + 1) += force(3 * ii + 1);
        forces(index + 2) += force(3 * ii + 2);
      }
    }

    return forces;
  }

  // UI Stuff - Maybe this should move
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

    if (drawPinned) {
      glColor3d(1.0, 0.0, 0.0);
      glPointSize(5.0);
      glBegin(GL_POINTS);
      for (int ii = 0; ii < pinned.size(); ++ii) {
        if (pinned(ii) == 1) {
          const Vec3<T> a = v.row(ii);
          glVertex3dv(a.data());
        }
      }
      glEnd();
    }
  }
  INLINE void ToggleDrawNormals() { drawNormals = !drawNormals; }
  INLINE void ToggleDrawPinned() { drawPinned = !drawPinned; }

  // Trivial Getters/Setters
  INLINE auto Positions() -> Vec<T> {
    Vec<T> out(DOFs());
    for (int ii = 0; ii < v.rows(); ++ii) {
      out[3 * ii] = v(ii, 0);
      out[3 * ii + 1] = v(ii, 1);
      out[3 * ii + 2] = v(ii, 2);
    }

    return out;
  }
  INLINE auto Displacements() -> Vec<T> { return Flatten(v - rv); }
  INLINE void SetPositions(const Vec<T> &x) {
    ASSERT(x.size() == DOFs(), "x.size(): " + std::to_string(x.size()) +
                                   " DOFs(): " + std::to_string(DOFs()));
    for (int ii = 0; ii < v.rows(); ++ii) {
      v(ii, 0) = x[3 * ii];
      v(ii, 1) = x[3 * ii + 1];
      v(ii, 2) = x[3 * ii + 2];
    }
  }
  INLINE void SetDisplacement(const Vec<T> &u) {
    v = rv + UnFlatten(u, v.rows(), v.cols());
  }
  INLINE void AddDisplacement(const Vec<T> &u) {
    v += UnFlatten(u, v.rows(), v.cols());
  }
  INLINE void PinVertex(int index) { pinned(index) = 1; }
  INLINE void UnPinVertex(int index) { pinned(index) = 0; }
  INLINE auto DOFs() -> int { return v.rows() * 3; }
  INLINE auto OneRingArea(int ii) -> T { return oneRingAreas.at(ii); }

  // Sim initializers
  INLINE static auto EvalPartialFPartialx(int index, const Mat3<T> &dmInv)
      -> Mat3<T> {
    ASSERT(index >= 0 && index <= 11, "Index: " + std::to_string(index) +
                                          " invalid. Must be in range[0, 11].");

    Mat3<T> ret = Mat3<T>::Zero();
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
    default:
      ASSERT(ASSERT_ALWAYS_FALSE_V, "Unreachable");
    }
    return ret * dmInv;
  }
  INLINE static auto PartialFPartialx(const Mat3<T> &dmInv) -> Mat9x12<T> {
    Mat9x12<T> ret = Mat9x12<T>::Zero();
    for (int ii = 0; ii < 12; ++ii) {
      ret.col(ii) = Flatten(EvalPartialFPartialx(ii, dmInv));
    }
    return ret;
  }
  INLINE static auto TetVolume(const Vec3<T> &a, const Vec3<T> &b,
                               const Vec3<T> &c, const Vec3<T> &d) -> T {
    const auto d1 = b - a;
    const auto d2 = c - a;
    const auto d3 = d - a;
    return d3.dot(d1.cross(d2)) / 6.0;
  }

private:
  bool drawNormals = false;
  bool drawPinned = true;

  // Per-element dm-inverses (for computing the deformation gradients)
  std::vector<Mat<T>> dmInvs;
  std::vector<Mat9x12<T>> partialFPartialxs;

  std::vector<T> volumes;
  std::vector<T> oneRingAreas;

  void InitializeDataStructures() {
    rv = v;
    pinned = Vec<T>::Zero(this->v.rows());

    dmInvs.resize(t.rows());
    partialFPartialxs.resize(t.rows());
    volumes.resize(t.rows());
    oneRingAreas.resize(v.rows());
    fs.resize(t.rows());

    // Precompute the Dm Inverses
    ComputeDmInverses();

    // Precompute the change-of-basis matrices
    ComputePartialFPartialxs();

    // Compute the tet volumes
    ComputeTetVolumes();

    // Compute the one-ring areas
    ComputeVertexAreas();

    BuildMassMatrix();
  }

  void Tetrahedralize() {
    static const std::string switches = "zpQ";

    igl::copyleft::tetgen::tetrahedralize(Mat<T>(v), Mat<int>(f), switches, v,
                                          t, f);
    igl::boundary_facets(t, f);

    // The faces come out of this function in the wrong winding order. So
    // this fixes that.
    f.rowwise().reverseInPlace();
  }

  INLINE void BuildMassMatrix() {
    m.resize(DOFs(), DOFs());
    mInv.resize(DOFs(), DOFs());
    m.setZero();
    mInv.setZero();

    std::vector<Eigen::Triplet<Real>> mTriplet;
    std::vector<Eigen::Triplet<Real>> mInvTriplet;
    for (int ii = 0; ii < v.rows(); ++ii) {
      const auto &oneRing = oneRingAreas.at(ii);
      const int index = ii * 3;
      mTriplet.emplace_back(index, index, oneRing);
      mTriplet.emplace_back(index + 1, index + 1, oneRing);
      mTriplet.emplace_back(index + 2, index + 2, oneRing);
      mInvTriplet.emplace_back(index, index, 1.0 / oneRing);
      mInvTriplet.emplace_back(index + 1, index + 1, 1.0 / oneRing);
      mInvTriplet.emplace_back(index + 2, index + 2, 1.0 / oneRing);
    }

    m.setFromTriplets(mTriplet.begin(), mTriplet.end());
    mInv.setFromTriplets(mInvTriplet.begin(), mInvTriplet.end());
  }
  void BuildRayleighDampingMatrix() {}

  INLINE void ComputeTetVolumes() {
    for (int ii = 0; ii < t.rows(); ++ii) {
      volumes.at(ii) = TetVolume(v.row(t(ii, 0)), v.row(t(ii, 1)),
                                 v.row(t(ii, 2)), v.row(t(ii, 3)));
      // Prevent negative volumes
      ASSERT2(volumes.at(ii) >= 0.0);
    }
  }

  INLINE void ComputeVertexAreas() {
    for (int ii = 0; ii < t.rows(); ++ii) {
      const Vec4<int> tet = t.row(ii);
      const Vec3<T> a = v.row(tet(0));
      const Vec3<T> b = v.row(tet(1));
      const Vec3<T> c = v.row(tet(2));
      const Vec3<T> d = v.row(tet(3));
      const T volume = TetVolume(a, b, c, d);
      oneRingAreas.at(tet(0)) += volume / 4.0;
      oneRingAreas.at(tet(1)) += volume / 4.0;
      oneRingAreas.at(tet(2)) += volume / 4.0;
      oneRingAreas.at(tet(3)) += volume / 4.0;
    }
  }

  INLINE void ComputeDmInverses() {
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
      dmInvs.at(ii) = dm.inverse();
    }
  }

  INLINE void ComputePartialFPartialxs() {
    for (int ii = 0; ii < t.rows(); ++ii) {
      partialFPartialxs.at(ii) = PartialFPartialx(dmInvs[ii]);
    }
  }
};
