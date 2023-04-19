#pragma once

#include "LibMath.h"
#include "OpenGL.h"
#include "Settings.h"
#include <Energy/Volume/HyperelasticMaterial.h>
#include <igl/boundary_facets.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>

constexpr std::array<float, 4> gAmbientFull = {{0.4, 0.4, 0.4, 1.0}};
constexpr std::array<float, 4> gSpecularFull = {1.0, 1.0, 1.0, 1.0};
constexpr std::array<float, 1> gShininess = {50.0};
constexpr std::array<float, 4> gDiffuseGray = {0.8, 0.8, 0.8, 1.0};

class TetMesh {
public:
  // Simulated vertices
  Mat<Real> v;

  // Rest vertices
  Mat<Real> rv;

  // Surface faces
  Mat<int> f;
  Mat<int> t;

  // Vertex normals
  Mat<Real> n;

  // Mass Matrix
  SparseMat<Real> m;
  SparseMat<Real> mInv;

  // Rayleigh Damping
  SparseMat<Real> rayleighDamping;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<int> pinned;

  // Per-element deformation gradients
  std::vector<Mat<Real>> fs;

  explicit TetMesh(const fs::path &file) {
    igl::read_triangle_mesh(file.string(), v, f);
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
    InitializeDataStructures();
  }
  TetMesh(const Mat<Real> &v, const Mat<int> &f) : v(v), f(f) {
    Tetrahedralize();
    igl::per_vertex_normals(v, f, n);
    InitializeDataStructures();
  }
  TetMesh(const Mat<Real> &v, const Mat<int> &f, const Mat<int> &t)
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
      const Vec3<Real> a = v.row(tet(0));
      const Vec3<Real> b = v.row(tet(1));
      const Vec3<Real> c = v.row(tet(2));
      const Vec3<Real> d = v.row(tet(3));
      Mat<Real> ds(3, 3);
      ds.col(0) = b - a;
      ds.col(1) = c - a;
      ds.col(2) = d - a;
      fs.at(ii) = ds * dmInvs[ii];
    }

    needsNewDeformationGradients = false;
  }

  INLINE auto
  ComputeMaterialForces(const std::shared_ptr<HyperelasticMaterial> &material)
      -> Vec<Real> {
    ASSERT2(!needsNewDeformationGradients);
    std::vector<Vec12<Real>> perElementForces(t.rows());
    for (int tt = 0; tt < t.rows(); ++tt) {
      const Mat3<Real> &F = fs[tt];
      const Mat3<Real> P = material->Pk1(F);
      const Vec12<Real> forceDensity =
          partialFPartialxs.at(tt).transpose() * ColwiseFlatten<Real>(P);
      const Vec12<Real> force = -restVolumes.at(tt) * forceDensity;
      perElementForces.at(tt) = force;
    }

    // Scatter global forces
    Vec<Real> forces = Vec<Real>::Zero(DOFs());
    for (int tt = 0; tt < t.rows(); ++tt) {
      const Vec4<int> tet = t.row(tt);
      const Vec12<Real> force = perElementForces.at(tt);

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
  void Draw();
  INLINE void ToggleDrawNormals() { drawNormals = !drawNormals; }
  INLINE void ToggleDrawPinned() { drawPinned = !drawPinned; }

  // Trivial Getters/Setters
  [[nodiscard]] INLINE auto Positions() const -> Vec<Real> {
    return RowwiseFlatten(v);
  }
  INLINE auto Displacements() -> Vec<Real> {
    return RowwiseFlatten<Real>(v - rv);
  }
  INLINE void SetPositions(const Vec<Real> &x) {
    v = RowwiseUnFlatten(x, v.rows(), v.cols());
    needsNewDeformationGradients = true;
  }
  INLINE void SetDisplacement(const Vec<Real> &u) {
    v = rv + RowwiseUnFlatten(u, v.rows(), v.cols());
    needsNewDeformationGradients = true;
  }
  INLINE void AddDisplacement(const Vec<Real> &u) {
    v += RowwiseUnFlatten(u, v.rows(), v.cols());
    needsNewDeformationGradients = true;
  }
  INLINE void PinVertex(int index) { pinned(index) = 1; }
  INLINE void UnPinVertex(int index) { pinned(index) = 0; }
  INLINE auto DOFs() const -> int { return v.rows() * 3; }
  INLINE auto OneRingArea(int ii) -> Real { return oneRingAreas.at(ii); }

  // Sim initializers
  INLINE static auto EvalPartialFPartialx(int index, const Mat3<Real> &dmInv)
      -> Mat3<Real> {
    ASSERT(index >= 0 && index <= 11, "Index: " + std::to_string(index) +
                                          " invalid. Must be in range[0, 11].");

    Mat3<Real> ret = Mat3<Real>::Zero();
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
  INLINE static auto PartialFPartialx(const Mat3<Real> &dmInv)
      -> Mat9x12<Real> {
    Mat9x12<Real> ret = Mat9x12<Real>::Zero();
    for (int ii = 0; ii < 12; ++ii) {
      ret.col(ii) = ColwiseFlatten<Real>(EvalPartialFPartialx(ii, dmInv));
    }
    return ret;
  }
  INLINE static auto TetVolume(const Vec3<Real> &a, const Vec3<Real> &b,
                               const Vec3<Real> &c, const Vec3<Real> &d)
      -> Real {
    const auto d1 = b - a;
    const auto d2 = c - a;
    const auto d3 = d - a;
    return d3.dot(d1.cross(d2)) / 6.0;
  }

private:
  bool drawNormals = false;
  bool drawPinned = true;

  bool needsNewDeformationGradients = false;

  // Per-element dm-inverses (for computing the deformation gradients)
  std::vector<Mat<Real>> dmInvs;
  std::vector<Mat9x12<Real>> partialFPartialxs;

  std::vector<Real> restVolumes;
  std::vector<Real> oneRingAreas;

  /**
   * @brief Initialize the data structures for the simulation.
   */
  void InitializeDataStructures();

  /**
   * @brief Tetrahedralize this mesh.
   */
  void Tetrahedralize();

  /**
   * @brief Build the mass matrix for the tetrahedral mesh.
   */
  void BuildMassMatrix();

  /**
   * @brief Build the damping matrix for the tetrahedral mesh.
   */
  void BuildRayleighDampingMatrix() {}

  /**
   * @brief Compute the rest volume for each tetrahedron.
   */
  void ComputeTetRestVolumes();

  /**
   * @brief Compute the one-ring area for each vertex.
   */
  void ComputeVertexAreas();

  /**
   * @brief Compute part of the deformation gradients for each tetrahedron.
   */
  void ComputeDmInverses();

  /**
   * @brief Computes the change-of-basis matrices for each tetrahedron.
   */
  void ComputePartialFPartialxs();
};
