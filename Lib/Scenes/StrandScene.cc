#include "StrandScene.h"
#include <igl/parallel_for.h>

void StrandScene::Step(const Vec3<Real> &gravity) {
  igl::parallel_for(integrators.size(), [&](int ii) {
    auto &integrator = integrators.at(ii);
    integrator->AddGravity(gravity);
    integrator->Step();
  });
}

void StrandScene::Draw() {
  for (const auto &mesh : meshes) {
    mesh->Draw();
  }
}

void StrandScene::DumpFrame() {
  char filename[512];
  sprintf(filename, "frame_%04i.obj", frame);

  std::ofstream file;
  file.open(filename);

  for (const auto &mesh : meshes) {
    const auto &vertices = mesh->der->vertices;
    for (int jj = 0; jj < vertices.rows(); ++jj) {
      // Write vertices.row(ii) to the file
      file << "v " << vertices.row(jj) << std::endl;
    }
  }

  for (int ii = 0; ii < meshes.size(); ++ii) {
    const auto &mesh = meshes.at(ii);
    const auto &vertices = mesh->der->vertices;
    file << "l ";
    for (int jj = 0; jj < vertices.rows(); ++jj) {
      int index = (jj + 1) + (ii * vertices.rows());
      file << index << " ";
    }

    file << std::endl;
  }

  file.close();
}

DiscreteElasticRods::DiscreteElasticRods(
    std::shared_ptr<Camera<float>> &camera) {
  for (Real ss = 0; ss < 20; ss += 0.1) {
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
        std::make_unique<ForwardEulerStrand>(mesh, nullptr, 1.0 / 1'000.0);
    meshes.emplace_back(mesh);
    integrators.emplace_back(std::move(integrator));
  }

  // Zoom out and set the center in a different spot
  camera->SetRadius(39.5999);
  camera->SetTheta(1.0108);
  camera->SetPhi(1.6308);
}
