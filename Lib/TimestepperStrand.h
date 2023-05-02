#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>
#include <StrandMesh.h>

class TimestepperStrand {
public:
  std::vector<VertexFaceCollision> vertexFaceCollisions;

  TimestepperStrand(std::shared_ptr<StrandMesh> strandMesh,
                    std::shared_ptr<DiscreteElasticRod> material,
                    Real dt = 1.0 / 3000.0)
      : strandMesh(std::move(strandMesh)), material(std::move(material)),
        dt(dt), externalForce(Vec<Real>::Zero(this->strandMesh->DOFs())),
        velocity(Vec<Real>::Zero(this->strandMesh->DOFs())) {}

  virtual void Step() = 0;
  INLINE void AddGravity(const Vec3<Real> &gravity) {
    for (int ii = 0; ii < strandMesh->der->vertices.rows(); ++ii) {
      this->externalForce.template segment<3>(3 * ii) = gravity;
    }
  }

protected:
  std::shared_ptr<StrandMesh> strandMesh;
  std::shared_ptr<DiscreteElasticRod> material;

  Real dt;

  // Externally applied forces
  Vec<Real> externalForce;

  // Current velocity
  Vec<Real> velocity;
};
