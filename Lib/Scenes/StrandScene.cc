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
  char dirname[512];
  sprintf(dirname, "sim_output_frame_%04i", frame);
  if (fs::exists(dirname)) {
    fs::remove_all(dirname);
  }

  // Create a directory for this frame
  fs::create_directory(dirname);

  int strand = 0;
  for (const auto &mesh : meshes) {
    char strandName[512];
    sprintf(strandName, "strand_%04i.obj", strand);
    fs::path fullPath = fs::path(dirname) / fs::path(strandName);

    std::ofstream file;
    file.open(fullPath);

    const auto &vertices = mesh->der->vertices;
    for (int ii = 0; ii < vertices.rows(); ++ii) {
      // Write vertices.row(ii) to the file
      file << "v " << vertices.row(ii) << std::endl;
    }

    file << "l ";
    for (int ii = 0; ii < vertices.rows(); ++ii) {
      file << ii + 1 << " ";
    }

    file << std::endl;
    file.close();

    ++strand;
  }
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
  camera->SetRadius(26.1);
  camera->SetTheta(1.5708);
  camera->SetPhi(1.0308);
}
