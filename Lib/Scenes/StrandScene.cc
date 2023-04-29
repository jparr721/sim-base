#include "StrandScene.h"

void StrandScene::Step(const Vec3<Real> &gravity) {
  integrator->AddGravity(gravity);
  integrator->Step();
}

void StrandScene::Draw() { mesh->Draw(); }

DiscreteElasticRods::DiscreteElasticRods(
    std::shared_ptr<Camera<float>> &camera) {
  // Construct a trivial point set
  std::vector<Vec3<Real>> points;
  for (Real ii = 0; ii < 10; ii += 0.5) {
    points.emplace_back(ii, 0, 0);
  }

  Mat<Real> v(points.size(), 3);
  for (int ii = 0; ii < points.size(); ++ii) {
    v.row(ii) = points.at(ii);
  }

  mesh = std::make_shared<StrandMesh>(v);
  integrator =
      std::make_unique<ForwardEulerStrand>(mesh, nullptr, 1.0 / 100'000.0);

  camera->SetRadius(26.1);
  camera->SetTheta(1.5708);
  camera->SetPhi(1.0308);
}
