#pragma once

#include <Energy/Volume/HyperelasticMaterial.h>
#include <TetMesh.h>
#include <Timestepper.h>

struct VolumeScene {
  std::shared_ptr<TetMesh> mesh;
  std::shared_ptr<HyperelasticMaterial> material;
  std::shared_ptr<Timestepper> integrator;

  void Step(const Vec3<Real> &gravity);
  void Draw();
};

// Coarse Bunny Scene
struct CoarseBunnyExplicit : public VolumeScene {
  CoarseBunnyExplicit();
};