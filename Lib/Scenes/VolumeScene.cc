#include "VolumeScene.h"
#include <Energy/Volume/SNH.h>
#include <ForwardEuler.h>

void VolumeScene::Step(const Vec3<Real> &gravity) {
  integrator->AddGravity(gravity);
  integrator->Step();
}

void VolumeScene::Draw() { mesh->Draw(); }

CoarseBunnyExplicit::CoarseBunnyExplicit() {
  mesh = std::make_shared<TetMesh>(Meshes / "bunny.obj");
  material = std::make_shared<SNH>(30.0, 0.45);

  // std::shared_ptr<STVK> gMaterial = std::make_shared<STVK>(30.0, 0.45);
  integrator = std::make_unique<ForwardEuler>(mesh, material, 1.0 / 3000.0);

  // Pin the top vertices of the mesh in mesh
  for (int ii = 0; ii < mesh->v.rows(); ++ii) {
    //     Find the top vertices
    if (mesh->v(ii, 1) > 0.9) {
      mesh->PinVertex(ii);
    }
    //    if (gMesh->v(ii, 1) > 0.65) {
    //      gMesh->PinVertex(ii);
    //    }
  }
}
