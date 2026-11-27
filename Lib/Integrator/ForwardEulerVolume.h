#pragma once

#include "VolumeIntegrator.h"
#include <Solid/ElasticSystem.h>

namespace sim {

class ForwardEulerVolume : public VolumeIntegrator {
public:
  // rayleighAlpha is the system's drag. rayleighBeta is unused.
  ForwardEulerVolume(std::shared_ptr<TetMesh> tetMesh,
                     Material material,
                     Real dt = 1.0 / 3000.0, Real rayleighAlpha = 0.0,
                     Real rayleighBeta = 0.0, ContactModel contact = {})
      : VolumeIntegrator(std::move(tetMesh), std::move(material), dt, rayleighAlpha,
                    rayleighBeta),
        system(*this->tetMesh, this->material, contact, this->rayleighAlpha),
        massInverse(ConstructDiagonalSparseMatrix<Real>(
            Vec<Real>(system.Mass().diagonal()).cwiseInverse())),
        filter(system.Constraints().Filter()) {}

  // Contacts the next Step() includes.
  void SetContacts(std::vector<SoftContact> contacts) {
    system.SetContacts(std::move(contacts));
  }

  SIM_INLINE void Step() override {
    const Vec<Real> x = this->tetMesh->Positions();
    const Vec<Real> R = system.Forces(x, this->velocity);

    // dx = S (dt^2 M^-1 f + dt v), then v = S dx / dt.
    Vec<Real> dx = (this->dt * this->dt) * massInverse *
                       (R + this->externalForce) +
                   this->dt * this->velocity;
    dx = filter * dx;
    this->tetMesh->SetPositions(x + dx);
    this->velocity = filter * Vec<Real>(dx / this->dt);
  }

private:
  ElasticSystem system;
  SparseMat<Real> massInverse;
  SparseMat<Real> filter;
};

} // namespace sim
