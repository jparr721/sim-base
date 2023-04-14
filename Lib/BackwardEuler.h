#pragma once

#include "Timestepper.h"

template <typename T> class BackwardEuler : public Timestepper<T> {
public:
  BackwardEuler(std::shared_ptr<TetMesh<T>> tetMesh,
                std::shared_ptr<HyperelasticMaterial> material,
                T dt = 1.0 / 60.0, T rayleighAlpha = 0.0, T rayleighBeta = 0.0)
      : Timestepper<T>(std::move(tetMesh), std::move(material), dt,
                       rayleighAlpha, rayleighBeta) {}

  INLINE void Step() override {

  }

private:
};
