#pragma once

#include "Timestepper.h"

template <typename T> class ForwardEuler : public Timestepper<T> {
public:
  ForwardEuler(std::shared_ptr<TetMesh> tetMesh,
               std::shared_ptr<HyperelasticMaterial> material,
               T dt = 1.0 / 300.0, T rayleighAlpha = 0.0, T rayleighBeta = 0.0)
      : Timestepper<T>(std::move(tetMesh), std::move(material), dt,
                       rayleighAlpha, rayleighBeta) {}

  INLINE void Step() override {
    // Get new deformation gradients
    this->tetMesh->ComputeDeformationGradients();

    // Compute the forces
    const Vec<T> &R = this->tetMesh->ComputeMaterialForces(this->material);

    // If in debug, print out R norm to get an idea of the intensity
#ifndef NDEBUG
    std::cout << "R norm: " << R.norm() << std::endl;
    std::cout << "Velocity norm: " << this->velocity.norm() << std::endl;
#endif

    // Compute the update
    Vec<T> u = (this->dt * this->dt) * this->tetMesh->mInv *
                   (R + this->externalForce) +
               this->dt * this->velocity;

    Vec<T> perVertexPinned = Vec<T>::Ones(this->tetMesh->DOFs());
    for (int ii = 0; ii < this->tetMesh->pinned.size(); ++ii) {
      const auto pinned = this->tetMesh->pinned(ii);
      if (pinned == 1) {
        perVertexPinned(3 * ii) = 0;
        perVertexPinned(3 * ii + 1) = 0;
        perVertexPinned(3 * ii + 2) = 0;
      }
    }

    SparseMat<T> filter = ConstructSparseMatrix(perVertexPinned);
    //        SparseMat<T>::Identity(this->tetMesh->DOFs(),
    //        this->tetMesh->DOFs());

//    for (int ii = 0; ii < this->tetMesh->pinned.size(); ++ii) {
//      if (this->tetMesh->pinned(ii) == 1) {
//        filter.template block<3, 3>(3 * ii, 3 * ii) = Mat3<T>::Zero();
//      }
//    }

    Vec<T> positions = this->tetMesh->Positions();
    u = filter * u;
    Vec<T> dx = u;
    u += positions;

    this->tetMesh->SetPositions(u);

    // Update the velocity
    this->velocity = dx / this->dt;
    this->velocity = filter * this->velocity;
  }
};
