#pragma once

#include <Energy/Collision/VertexFaceSqrtCollision.h>
#include <Energy/Damping/Damping.h>
#include <Energy/Strand/DiscreteElasticRod.h>
#include <Solid/Constraints.h>
#include <Solid/Contact.h>
#include <Solid/System.h>

#include <vector>

namespace sim {

// One strand as a System over its vertex positions, 3 DOFs per vertex and
// no theta (see DiscreteElasticRod). The thetas are read from the rod and
// held through a step, so Forces includes the twist force at the step's end
// angles. That force is not the gradient of an energy in x: Energy leaves
// its work out, and Stiffness's twist blocks are its Jacobian, which is not
// symmetric. Clamping symmetrizes them.
//
// A contact is the collision energy, scaled by its triangle's area, of a
// vertex near a static collider triangle. damping is NoDamping or Rayleigh,
// whose stiffness term uses the clamped elastic stiffness without contacts.
// The rod must outlive the system.
class DerSystem {
public:
  DerSystem(const DiscreteElasticRod &rod, const std::vector<int> &pinned,
            Damping damping, Real collisionStiffness, Real collisionEpsilon);
  DerSystem(DiscreteElasticRod &&, const std::vector<int> &, Damping, Real,
            Real) = delete;

  // The contacts every later evaluation includes, until the next call.
  void SetContacts(std::vector<SoftContact> found) {
    contacts = std::move(found);
  }
  void SetDamping(Damping replacement) { damping = std::move(replacement); }

  [[nodiscard]] auto Dofs() const -> int { return rod.DOFs(); }
  [[nodiscard]] auto Mass() const -> const SparseMat<Real> & { return mass; }
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
  const DiscreteElasticRod &rod;
  Damping damping;
  VertexFaceSqrtCollision collision;
  SparseMat<Real> mass;
  sim::Constraints constraints;
  std::vector<SoftContact> contacts;

  // Bending, twist and springs, without the contacts.
  [[nodiscard]] auto ElasticStiffness(const Vec<Real> &x, bool clamped) const
      -> std::vector<Eigen::Triplet<Real>>;
};

static_assert(System<DerSystem>);

} // namespace sim
