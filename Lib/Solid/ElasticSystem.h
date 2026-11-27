#pragma once

#include <Energy/Volume/Material.h>
#include <Solid/Constraints.h>
#include <Solid/System.h>
#include <TetMesh.h>

namespace sim {

// Global assembly over one TetMesh: the one place per-tet quantities scatter
// into DOF-sized vectors and matrices. Both references must outlive it, and
// the mesh's pins are read once, here.
class ElasticSystem {
public:
  ElasticSystem(const TetMesh &mesh, const Material &material);
  ElasticSystem(TetMesh &&, const Material &) = delete;
  ElasticSystem(const TetMesh &, Material &&) = delete;

  [[nodiscard]] auto Dofs() const -> int { return mesh.DOFs(); }
  [[nodiscard]] auto Mass() const -> const SparseMat<Real> & { return mesh.m; }
  [[nodiscard]] auto Forces(const Vec<Real> &x, const Vec<Real> &v) const
      -> Vec<Real>;
  [[nodiscard]] auto Stiffness(const Vec<Real> &x, bool clamped) const
      -> SparseMat<Real>;
  // All zeros until H4 adds a damping energy.
  [[nodiscard]] auto DampingMatrix(const Vec<Real> &x) const -> SparseMat<Real>;
  [[nodiscard]] auto Constraints() const -> const sim::Constraints & {
    return constraints;
  }

private:
  const TetMesh &mesh;
  const Material &material;
  sim::Constraints constraints;

  [[nodiscard]] auto DeformationGradient(const Vec<Real> &x, int tet) const
      -> Mat3<Real>;
};

static_assert(System<ElasticSystem>);

} // namespace sim
