#pragma once

#include <Energy/Collision/VertexFaceSqrtCollision.h>
#include <Energy/Damping/Damping.h>
#include <Energy/Volume/FiberMaterial.h>
#include <Energy/Volume/Material.h>
#include <Solid/Constraints.h>
#include <Solid/Contact.h>
#include <Solid/System.h>
#include <TetMesh.h>

#include <optional>
#include <vector>

namespace sim {

// Global assembly over one TetMesh: the one place per-tet and per-contact
// quantities scatter into DOF-sized vectors and matrices. Both references must
// outlive it, and the mesh's pins are read once, here. `damping` is the
// material's, and `drag` (1/s) is mass-proportional damping, -drag M v.
// `fiber`, when present, adds its energy density to the material's in every
// tet, with its direction in the rest mesh's frame. Every
// damping force is linear in v, so Forces(x, v) = Forces(x, 0) -
// DampingMatrix(x) v.
class ElasticSystem {
public:
  ElasticSystem(const TetMesh &mesh, const Material &material, Damping damping,
                ContactModel contact = {}, Real drag = 0,
                std::optional<FiberMaterial> fiber = {});
  ElasticSystem(TetMesh &&, const Material &, Damping) = delete;
  ElasticSystem(const TetMesh &, Material &&, Damping) = delete;

  // The contacts every later evaluation includes, until the next call.
  void SetContacts(std::vector<SoftContact> found) {
    contacts = std::move(found);
  }

  [[nodiscard]] auto Dofs() const -> int { return mesh.DOFs(); }
  [[nodiscard]] auto Mass() const -> const SparseMat<Real> & { return mesh.m; }
  [[nodiscard]] auto Energy(const Vec<Real> &x) const -> Real;
  [[nodiscard]] auto Forces(const Vec<Real> &x, const Vec<Real> &v) const
      -> Vec<Real>;
  [[nodiscard]] auto Stiffness(const Vec<Real> &x, bool clamped) const
      -> SparseMat<Real>;
  [[nodiscard]] auto DampingMatrix(const Vec<Real> &x) const -> SparseMat<Real>;
  [[nodiscard]] auto Constraints() const -> const sim::Constraints & {
    return constraints;
  }

private:
  const TetMesh &mesh;
  const Material &material;
  Damping damping;
  ContactModel contact;
  Real drag;
  std::optional<FiberMaterial> fiber;
  sim::Constraints constraints;
  std::vector<SoftContact> contacts;

  // Calls visit(density) for the material's density, then the fiber's.
  template <typename Visitor> void ForEachDensity(const Visitor &visit) const {
    std::visit(visit, material);
    if (fiber) {
      std::visit(visit, *fiber);
    }
  }

  [[nodiscard]] auto VertexMass(int vertex) const -> Real;
  [[nodiscard]] auto ContactEnergy(int vertex) const
      -> VertexFaceSqrtCollision;

  [[nodiscard]] auto DeformationGradient(const Vec<Real> &x, int tet) const
      -> Mat3<Real>;
  [[nodiscard]] auto RayleighMatrix(const Rayleigh &rayleigh,
                                    const Vec<Real> &x) const
      -> SparseMat<Real>;
  // The elastic terms of Stiffness, without the contacts.
  [[nodiscard]] auto ElasticStiffness(const Vec<Real> &x, bool clamped) const
      -> std::vector<Eigen::Triplet<Real>>;
};

static_assert(System<ElasticSystem>);

} // namespace sim
