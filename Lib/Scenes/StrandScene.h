#pragma once

#include <Camera.h>
#include <ForwardEulerStrand.h>
#include <StrandMesh.h>
#include <TimestepperVolume.h>
#include <memory>

struct StrandScene {
  std::vector<std::shared_ptr<StrandMesh>> meshes;
  std::vector<std::shared_ptr<ForwardEulerStrand>> integrators;

  void Step(const Vec3<Real> &gravity);
  void Draw();
};

struct DiscreteElasticRods : public StrandScene {
  DiscreteElasticRods(std::shared_ptr<Camera<float>> &camera);
};