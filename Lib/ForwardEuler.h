#pragma once

#include "Timestepper.h"

class ForwardEuler : public Timestepper {
public:
  ForwardEuler(std::shared_ptr<TetMesh> tetMesh,
               std::shared_ptr<HyperelasticMaterial> material)
      : Timestepper(std::move(tetMesh), std::move(material)) {}

  INLINE void Step() override {
    auto positions = tetMesh->FlatPositions();

  }
};
