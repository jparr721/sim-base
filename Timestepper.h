#pragma once

#include "HyperelasticMaterial.h"
#include "TetMesh.h"
#include <memory>
#include <utility>

class Timestepper {
public:
  Timestepper(std::shared_ptr<TetMesh> tetMesh,
              std::shared_ptr<HyperelasticMaterial> material)
      : tetMesh(std::move(tetMesh)), material(std::move(material)) {}

private:
  std::shared_ptr<TetMesh> tetMesh;
  std::shared_ptr<HyperelasticMaterial> material;
};
