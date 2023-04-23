#pragma once

#include <ForwardEulerStrand.h>
#include <StrandMesh.h>
#include <TimestepperVolume.h>
#include <memory>

struct StrandScene {
  std::shared_ptr<StrandMesh> mesh;
  std::shared_ptr<ForwardEulerStrand> integrator;

  void Step(const Vec3<Real> &gravity);
  void Draw();
};

struct DiscreteElasticRods : public StrandScene {
  DiscreteElasticRods();
};