#pragma once

// Headless replicas of the GLUT drivers' setup and idle loops, step for step.
// They produced Tests/Golden/*.txt and are the oracle the scene API is held to.

#include <Integrator/ForwardEulerStrand.h>
#include <Integrator/ForwardEulerVolume.h>
#include <Scenes/StrandDropScene.h>
#include <Scenes/VolumeScene.h>

#include "Golden.h"

namespace golden {

inline void RunBunnyDrop(const Run &run, const Recorder &record) {
  CoarseBunnyExplicit scene;
  const Vec3<Real> gravity(0, -0.981, 0);
  for (int step = 1; step <= run.steps; ++step) {
    scene.Step(gravity);
    if (step % run.checkpointEvery == 0) {
      record(step, {scene.mesh->v});
    }
  }
}

inline auto MakeLineStrand(int nVertices) -> std::shared_ptr<StrandMesh> {
  Mat<Real> v(nVertices, 3);
  for (int ii = 0; ii < v.rows(); ++ii) {
    v.row(ii) = Vec3<Real>(ii, 0, 0);
  }
  auto mesh = std::make_shared<StrandMesh>(v);
  mesh->pinned(0) = 1;
  mesh->pinned(v.rows() - 1) = 1;
  return mesh;
}

inline void RunStrandBend(const Run &run, const Recorder &record) {
  auto mesh = MakeLineStrand(5);
  ForwardEulerStrand integrator(mesh, nullptr, 1.0 / 1'000.0, 5, 0.3);
  const Vec<Real> start = Vec<Real>::LinSpaced(5000, 0, 2.5);
  const Vec<Real> end = Vec<Real>::LinSpaced(5000, -2.5, 0).reverse();
  int translationIndex = 0;
  for (int step = 1; step <= run.steps; ++step) {
    integrator.AddGravity(Vec3<Real>(0, -0.981, 0));
    integrator.Step();
    // Recorded before the pin move so the snapshot is the integrated state.
    if (step % run.checkpointEvery == 0) {
      record(step, {mesh->der->vertices});
    }
    if (step > 1000 && translationIndex < start.rows()) {
      mesh->TranslatePinnedX(start(translationIndex), 0);
      mesh->TranslatePinnedX(end(translationIndex), 4);
      ++translationIndex;
    }
  }
}

inline void RunStrandTwist(const Run &run, const Recorder &record) {
  auto mesh = MakeLineStrand(8);
  mesh->der->rightAngularVelocity = 50;
  mesh->der->leftAngularVelocity = 0;
  ForwardEulerStrand integrator(mesh, nullptr, 1.0 / 1'000.0, 5, 0.3);
  for (int step = 1; step <= run.steps; ++step) {
    integrator.AddGravity(Vec3<Real>(0, -0.981, 0));
    integrator.Step();
    if (step > 2'000) {
      mesh->der->rightAngularVelocity = 0;
      mesh->der->leftAngularVelocity = 0;
    }
    if (step % run.checkpointEvery == 0) {
      record(step, {mesh->der->vertices});
    }
  }
}

// The GLUT driver advanced `frame` every 50 steps from its idle callback, and
// StepScriptedScene keyed the comb sweep and damping off that counter.
inline void RunStrandDrop(const Run &run, const Recorder &record) {
  DiscreteElasticRods scene;
  const Vec3<Real> gravity(0, -0.981, 0);
  for (int step = 1; step <= run.steps; ++step) {
    scene.StepScriptedScene(gravity);
    if (step % 50 == 0) {
      ++scene.frame;
      if (scene.frame > 400) {
        ++scene.sceneFrames;
      }
    }
    if (step % run.checkpointEvery == 0) {
      Snapshot snapshot;
      for (const auto &mesh : scene.meshes) {
        snapshot.push_back(mesh->der->vertices);
      }
      snapshot.push_back(scene.comb->v);
      record(step, snapshot);
    }
  }
}

inline void RunLegacy(const Run &run, const Recorder &record) {
  if (run.name == "bunny-drop") return RunBunnyDrop(run, record);
  if (run.name == "strand-bend") return RunStrandBend(run, record);
  if (run.name == "strand-twist") return RunStrandTwist(run, record);
  if (run.name == "strand-drop") return RunStrandDrop(run, record);
}

} // namespace golden
