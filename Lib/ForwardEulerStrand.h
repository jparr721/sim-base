#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <StrandMesh.h>
#include <TimestepperStrand.h>

class ForwardEulerStrand : public TimestepperStrand {
public:
  ForwardEulerStrand(std::shared_ptr<StrandMesh> strandMesh,
                     std::shared_ptr<DiscreteElasticRod> material,
                     Real dt = 1.0 / 3000.0)
      : TimestepperStrand(std::move(strandMesh), std::move(material), dt) {}

  INLINE void Step() override {
    const Vec<Real> R = this->strandMesh->ComputeMaterialForces();

// If in debug, print out R norm to get an idea of the intensity
#ifndef NDEBUG
    std::cout << "R norm: " << R.norm() << std::endl;
    std::cout << "Velocity norm: " << this->velocity.norm() << std::endl;
#endif

    // Compute the update
    Vec<Real> u = (this->dt * this->dt) * this->strandMesh->der->mInv *
                      (R + this->externalForce) +
                  this->dt * this->velocity;

    Vec<Real> positions = this->strandMesh->Positions();
    u += positions;
    this->strandMesh->SetPositions(u);
    this->velocity = u / this->dt;

    this->strandMesh->der->UpdateBishopFrames();
    this->strandMesh->der->UpdateMaterialFrames();
  }
};