#include "StrandScene.h"

void StrandScene::Step(const Vec3<Real> &gravity) {
  integrator->AddGravity(gravity);
  integrator->Step();
}

void StrandScene::Draw() { mesh->Draw(); }

DiscreteElasticRods::DiscreteElasticRods() {
  // Construct a trivial point set
  std::vector<Vec3<Real>> points;
  for (Real ii = 0; ii < 3; ii += 0.5) {
    points.emplace_back(ii, ii * ii * 0.25, 0);
  }

  Mat<Real> v(points.size(), 3);
  for (int ii = 0; ii < points.size(); ++ii) {
    v.row(ii) = points.at(ii);
  }

  mesh = std::make_shared<StrandMesh>(v);
  integrator =
      std::make_unique<ForwardEulerStrand>(mesh, nullptr, 1.0 / 3000.0);
}
