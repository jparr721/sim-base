#pragma once

#include <Camera.h>
#include <Collision/CollisionMesh.h>
#include <Integrator/ForwardEulerStrand.h>
#include <Integrator/VolumeIntegrator.h>
#include <StrandMesh.h>
#include <memory>

struct Comb : public CollisionMesh {
  Comb();
};

struct StrandDropScene {
  bool stopped = false;
  Real collisionEnvelopeSize = 0.4;
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

struct DiscreteElasticRods : public StrandDropScene {
  explicit DiscreteElasticRods(std::shared_ptr<Camera<float>> &camera);
};