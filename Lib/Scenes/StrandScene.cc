#include "StrandScene.h"

void StrandScene::Step(const Vec3<Real> &gravity) {}

void StrandScene::Draw() { mesh->Draw(); }

DiscreteElasticRods::DiscreteElasticRods() {
  // Make a trivial point set of increasing values in the x direction from 1 -
  // 10
  Mat<Real> v(10, 3);
  for (int ii = 0; ii < 10; ++ii) {
    v.row(ii) = Vec3<Real>::UnitY() * ii;
  }

  mesh = std::make_shared<StrandMesh>(v);
}
