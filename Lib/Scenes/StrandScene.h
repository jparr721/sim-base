#pragma once

#include <Camera.h>
#include <ForwardEulerStrand.h>
#include <StrandMesh.h>
#include <TimestepperVolume.h>
#include <memory>

struct Comb : public CollisionMesh {
  Comb();
};

struct StrandScene {
  int frame = 0;
  int sceneFrames = 0;
  Vec<Real> combTranslation;
  std::shared_ptr<Comb> comb;
  std::vector<std::shared_ptr<StrandMesh>> meshes;
  std::vector<std::shared_ptr<ForwardEulerStrand>> integrators;
  std::vector<std::vector<VertexFaceCollision>> vertexFaceCollisionsPerStrand;

  void StepScriptedScene(const Vec3<Real> &gravity);
  void Step(const Vec3<Real> &gravity);
  void Draw();
  void DumpFrame();
};

struct DiscreteElasticRods : public StrandScene {
  explicit DiscreteElasticRods(std::shared_ptr<Camera<float>> &camera);
};