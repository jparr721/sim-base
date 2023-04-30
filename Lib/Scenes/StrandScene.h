#pragma once

#include <Camera.h>
#include <ForwardEulerStrand.h>
#include <StrandMesh.h>
#include <TimestepperVolume.h>
#include <memory>

struct StrandScene {
  int frame = 0;
  std::vector<std::shared_ptr<StrandMesh>> meshes;
  std::vector<std::shared_ptr<ForwardEulerStrand>> integrators;

  void Step(const Vec3<Real> &gravity);
  void Draw();
  void DumpFrame();
};

struct DiscreteElasticRods : public StrandScene {
  DiscreteElasticRods(std::shared_ptr<Camera<float>> &camera);
};