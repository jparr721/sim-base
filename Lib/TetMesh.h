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

  TetMesh() = default;

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

  void ComputeDeformationGradients();

  auto
  ComputeMaterialForces(const std::shared_ptr<HyperelasticMaterial> &material)
      -> Vec<Real>;

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
  [[nodiscard]] INLINE auto DOFs() const -> int { return v.rows() * 3; }
  INLINE auto OneRingArea(int ii) -> Real { return oneRingAreas.at(ii); }

  // Sim initializers
  auto EvalPartialFPartialx(int index, const Mat3<Real> &dmInv) -> Mat3<Real>;
  auto PartialFPartialx(const Mat3<Real> &dmInv) -> Mat9x12<Real>;
  auto TetVolume(const Vec3<Real> &a, const Vec3<Real> &b, const Vec3<Real> &c,
                 const Vec3<Real> &d) -> Real;

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
