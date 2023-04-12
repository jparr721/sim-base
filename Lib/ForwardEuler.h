#pragma once

#include "Timestepper.h"

template <typename T> class ForwardEuler : public Timestepper<T> {
public:
  ForwardEuler(std::shared_ptr<TetMesh<T>> tetMesh,
               std::shared_ptr<HyperelasticMaterial> material)
      : Timestepper<T>(std::move(tetMesh), std::move(material)) {}

  INLINE void Step() override {
    // Compute the forces
    const Vec<T> &R = this->tetMesh->ComputeMaterialForces(this->material);

    // If in debug, print out R norm to get an idea of the intensity
#ifndef N_DEBUG
    std::cout << "R norm: " << R.norm() << std::endl;
#endif

    // Compute the update
    Vec<T> u = (this->dt * this->dt) * this->tetMesh->mInv *
                   (R + this->externalForce + this->velocity) +
               this->dt * this->velocity;

    Mat<T> filter(this->tetMesh->DOFs(), this->tetMesh->DOFs());
    filter.setIdentity();

    for (int ii = 0; ii < this->tetMesh->pinned.size(); ++ii) {
      if (this->tetMesh->pinned(ii) == 1) {
        filter.template block<3, 3>(3 * ii, 3 * ii) = Mat3<T>::Zero();
      }
    }

    Vec<T> positions = this->tetMesh->FlatPositions();
    u = filter * u;
    u += positions;
    Vec<T> diff = u - positions;

    this->tetMesh->SetPositions(u);

    // Update the velocity
    this->velocity = diff / this->dt;
  }
};
