#include "StrandDropScene.h"
#include <igl/parallel_for.h>
#include <igl/readOBJ.h>

Comb::Comb() {
  igl::readOBJ(Meshes / "comb_more_teeth.obj", v, f);
  rv = v;
}

void StrandDropScene::StepScriptedScene(const Vec3<Real> &gravity) {
  if (sceneFrames > 200) {
    stopped = true;
    return;
  }

  if (frame > 400) {
    const Vec3<Real> translation(-1, combTranslation(sceneFrames), 10.0);
    comb->Translate(translation);
  }

  Step(gravity);
}

void StrandDropScene::Step(const Vec3<Real> &gravity) {
  // Compute collisions over all meshes
  vertexFaceCollisionsPerStrand.clear();
  vertexFaceCollisionsPerStrand.resize(meshes.size());

  igl::parallel_for(meshes.size(), [&](int ii) {
    auto &mesh = meshes.at(ii);
    vertexFaceCollisionsPerStrand.at(ii) =
        comb->DetectFaceCollisions(collisionEnvelopeSize, mesh->der->vertices);
  });

  igl::parallel_for(integrators.size(), [&](int ii) {
    auto &integrator = integrators.at(ii);
    integrator->vertexFaceCollisions = vertexFaceCollisionsPerStrand.at(ii);
    integrator->AddGravity(gravity);
    integrator->damping = sceneFrames > 0 ? 0.45 : 0.75;
    integrator->Step();
  });
}

DiscreteElasticRods::DiscreteElasticRods() {
  comb = std::make_shared<Comb>();
  comb->Translate(Vec3<Real>(-1, -1, 10));
  combTranslation = Vec<Real>::LinSpaced(200, -8, -1).reverse();

  for (Real ss = 8; ss < 12; ss += 0.05) {
    // Construct a trivial point set
    std::vector<Vec3<Real>> points;
    for (Real ii = 0; ii < 8; ii += 0.25) {
      points.emplace_back(ii, 0, ss);
    }

    Mat<Real> v(points.size(), 3);
    for (int ii = 0; ii < std::ssize(points); ++ii) {
      v.row(ii) = points.at(ii);
    }

    auto mesh = std::make_shared<StrandMesh>(v);
    mesh->pinned(0) = 1;
    auto integrator = std::make_unique<ForwardEulerStrand>(
        mesh, nullptr, 1.0 / 1'000.0, 5, 0.3);
    meshes.emplace_back(mesh);
    integrators.emplace_back(std::move(integrator));
  }
}
