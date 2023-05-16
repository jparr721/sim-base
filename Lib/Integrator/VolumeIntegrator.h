#pragma once

#include "TetMesh.h"
#include <Energy/Volume/HyperelasticMaterial.h>
#include <memory>
#include <utility>

class VolumeIntegrator {
public:
  VolumeIntegrator(std::shared_ptr<TetMesh> tetMesh,
                 std::shared_ptr<HyperelasticMaterial> material,
                 Real dt = 1.0 / 300.0, Real rayleighAlpha = 0.0,
                 Real rayleighBeta = 0.0)
      : tetMesh(std::move(tetMesh)), material(std::move(material)), dt(dt),
        rayleighAlpha(rayleighAlpha), rayleighBeta(rayleighBeta),
        externalForce(Vec<Real>::Zero(this->tetMesh->DOFs())),
        velocity(Vec<Real>::Zero(this->tetMesh->DOFs())) {}

  virtual void Step() = 0;
  INLINE void AddGravity(const Vec3<Real> &gravity) {
    for (int ii = 0; ii < tetMesh->v.rows(); ++ii) {
      const Real area = tetMesh->OneRingArea(ii);
      this->externalForce.template segment<3>(3 * ii) = area * gravity;
    }
  }

protected:
  std::shared_ptr<TetMesh> tetMesh;
  std::shared_ptr<HyperelasticMaterial> material;

  Real dt;
  Real rayleighAlpha;
  Real rayleighBeta;

  // Externally applied forces
  Vec<Real> externalForce;

  // Current velocity
  Vec<Real> velocity;
};
