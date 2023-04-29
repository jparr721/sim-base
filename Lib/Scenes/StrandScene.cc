#include "StrandScene.h"
#include <igl/parallel_for.h>

void StrandScene::Step(const Vec3<Real> &gravity) {

  //  igl::parallel_for(integrators.size(), [&](int ii) {
  for (int ii = 0; ii < integrators.size(); ++ii) {
    auto &integrator = integrators.at(ii);
    integrator->AddGravity(gravity);
    integrator->Step();
  }
  //  });
}

void StrandScene::Draw() {
  for (const auto &mesh : meshes) {
    mesh->Draw();
  }
}

DiscreteElasticRods::DiscreteElasticRods(
    std::shared_ptr<Camera<float>> &camera) {
  for (int ss = 0; ss < 10; ++ss) {
    // Construct a trivial point set
    std::vector<Vec3<Real>> points;
    for (int ii = 0; ii < 10; ++ii) {
      points.emplace_back(ii, 0, ss);
    }

    Mat<Real> v(points.size(), 3);
    for (int ii = 0; ii < points.size(); ++ii) {
      v.row(ii) = points.at(ii);
    }

    auto mesh = std::make_shared<StrandMesh>(v);
    auto integrator =
        std::make_unique<ForwardEulerStrand>(mesh, nullptr, 1.0 / 100'000.0);
    meshes.emplace_back(mesh);
    integrators.emplace_back(std::move(integrator));
  }

  // Zoom out and set the center in a different spot
  camera->SetRadius(26.1);
  camera->SetTheta(1.5708);
  camera->SetPhi(1.0308);
}
