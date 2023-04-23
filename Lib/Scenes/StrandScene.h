#pragma once

#include <StrandMesh.h>
#include <TimestepperVolume.h>
#include <memory>

struct StrandScene {
  std::shared_ptr<StrandMesh> mesh;

  void Step(const Vec3<Real> &gravity);
  void Draw();
};

struct DiscreteElasticRods : public StrandScene {
  DiscreteElasticRods();
};