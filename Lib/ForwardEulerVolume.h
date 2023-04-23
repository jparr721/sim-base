#pragma once

#include "TimestepperVolume.h"
#include <Energy/Volume/HyperelasticMaterial.h>

class ForwardEulerVolume : public TimestepperVolume {
public:
  ForwardEulerVolume(std::shared_ptr<TetMesh> tetMesh,
                     std::shared_ptr<HyperelasticMaterial> material,
                     Real dt = 1.0 / 3000.0, Real rayleighAlpha = 0.0,
                     Real rayleighBeta = 0.0)
      : TimestepperVolume(std::move(tetMesh), std::move(material), dt, rayleighAlpha,
                    rayleighBeta) {}

  INLINE void Step() override {
    // Get new deformation gradients
    this->tetMesh->ComputeDeformationGradients();

    // Compute the forces
    const Vec<Real> &R = this->tetMesh->ComputeMaterialForces(this->material);

    // If in debug, print out R norm to get an idea of the intensity
#ifndef NDEBUG
    std::cout << "R norm: " << R.norm() << std::endl;
    std::cout << "Velocity norm: " << this->velocity.norm() << std::endl;
#endif

    // Compute the update
    Vec<Real> u = (this->dt * this->dt) * this->tetMesh->mInv *
                      (R + this->externalForce) +
                  this->dt * this->velocity;

    Vec<Real> perVertexPinned = Vec<Real>::Ones(this->tetMesh->DOFs());
    for (int ii = 0; ii < this->tetMesh->pinned.size(); ++ii) {
      const auto pinned = this->tetMesh->pinned(ii);
      if (pinned == 1) {
        perVertexPinned(3 * ii) = 0;
        perVertexPinned(3 * ii + 1) = 0;
        perVertexPinned(3 * ii + 2) = 0;
      }
    }

    SparseMat<Real> filter = ConstructDiagonalSparseMatrix(perVertexPinned);
    Vec<Real> positions = this->tetMesh->Positions();
    u = filter * u;
    Vec<Real> dx = u;
    u += positions;

    this->tetMesh->SetPositions(u);

    // Update the velocity
    this->velocity = dx / this->dt;
    this->velocity = filter * this->velocity;
  }
};
