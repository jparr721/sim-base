#pragma once

#include "HyperelasticMaterial.h"
#include "TetMesh.h"
#include <memory>
#include <utility>

template <typename T> class Timestepper {
public:
  Timestepper(std::shared_ptr<TetMesh<T>> tetMesh,
              std::shared_ptr<HyperelasticMaterial> material,
              T dt = 1.0 / 300.0, T rayleighAlpha = 0.0, T rayleighBeta = 0.0)
      : tetMesh(std::move(tetMesh)), material(std::move(material)), dt(dt),
        rayleighAlpha(rayleighAlpha), rayleighBeta(rayleighBeta),
        externalForce(Vec<T>::Zero(this->tetMesh->DOFs())),
        velocity(Vec<T>::Zero(this->tetMesh->DOFs())) {}

  virtual void Step() = 0;
  INLINE void AddGravity(const Vec3<T> &gravity) {
    for (int ii = 0; ii < tetMesh->v.rows(); ++ii) {
      const auto area = tetMesh->OneRingArea(ii);
      this->externalForce.template segment<3>(3 * ii) = area * gravity;
    }
  }

protected:
  std::shared_ptr<TetMesh<T>> tetMesh;
  std::shared_ptr<HyperelasticMaterial> material;

  T dt;
  T rayleighAlpha;
  T rayleighBeta;

  // Externally applied forces
  Vec<T> externalForce;

  // Current velocity
  Vec<T> velocity;
};
