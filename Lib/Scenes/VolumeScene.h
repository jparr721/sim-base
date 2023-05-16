#pragma once

#include <Energy/Volume/HyperelasticMaterial.h>
#include <TetMesh.h>
#include <Integrator/VolumeIntegrator.h>

struct VolumeScene {
  std::shared_ptr<TetMesh> mesh;
  std::shared_ptr<HyperelasticMaterial> material;
  std::shared_ptr<VolumeIntegrator> integrator;

  void Step(const Vec3<Real> &gravity);
  void Draw();
};

// Coarse Bunny Scene
struct CoarseBunnyExplicit : public VolumeScene {
  CoarseBunnyExplicit();
};