#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>
#include <StrandMesh.h>

namespace sim {

class StrandIntegrator {
public:
  virtual ~StrandIntegrator() = default;
  Real dt;
  // Velocity damping coefficient applied after each step.
  Real damping = 0.75;
  std::vector<VertexFaceCollision> vertexFaceCollisions;

  StrandIntegrator(std::shared_ptr<StrandMesh> strandMesh,
                    std::shared_ptr<DiscreteElasticRod> material,
                    Real dt = 1.0 / 3000.0)
      : dt(dt), strandMesh(std::move(strandMesh)),
        material(std::move(material)), externalForce(Vec<Real>::Zero(this->strandMesh->DOFs())),
        velocity(Vec<Real>::Zero(this->strandMesh->DOFs())) {}

  virtual void Step() = 0;
  SIM_INLINE void AddGravity(const Vec3<Real> &gravity) {
    for (int ii = 0; ii < strandMesh->der->vertices.rows(); ++ii) {
      this->externalForce.template segment<3>(3 * ii) = gravity;
    }
  }

protected:
  std::shared_ptr<StrandMesh> strandMesh;
  std::shared_ptr<DiscreteElasticRod> material;

  // Externally applied forces
  Vec<Real> externalForce;

  // Current velocity
  Vec<Real> velocity;
};

} // namespace sim
