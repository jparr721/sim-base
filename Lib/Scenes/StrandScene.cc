#include "StrandScene.h"
#include <LibGui.h>
#include <igl/parallel_for.h>
#include <igl/readOBJ.h>

Comb::Comb() {
  igl::readOBJ(Meshes / "comb.obj", v, f);
  rv = v;
}

void StrandScene::StepScriptedScene(const Vec3<Real> &gravity) {
  if (sceneFrames > 200) {
    return;
  }

  if (frame > 400) {
    const Vec3<Real> translation(-1, combTranslation(sceneFrames), 10.0);
    comb->Translate(translation);
  }

  Step(gravity);
}

void StrandScene::Step(const Vec3<Real> &gravity) {
  // Compute collisions over all meshes
  vertexFaceCollisionsPerStrand.clear();
  vertexFaceCollisionsPerStrand.resize(meshes.size());

  igl::parallel_for(meshes.size(), [&](int ii) {
    auto &mesh = meshes.at(ii);
    vertexFaceCollisionsPerStrand.at(ii) =
        comb->DetectFaceCollisions(0.5, mesh->der->vertices);
  });

  igl::parallel_for(integrators.size(), [&](int ii) {
    auto &integrator = integrators.at(ii);
    integrator->vertexFaceCollisions = vertexFaceCollisionsPerStrand.at(ii);
    integrator->AddGravity(gravity);
    integrator->Step();
  });
}

void StrandScene::Draw() {
  for (const auto &mesh : meshes) {
    mesh->Draw();
  }

  // Combine vertexFaceCollisionsPerStrand into one vector
  std::vector<VertexFaceCollision> vertexFaceCollisions;
  for (const auto &collisions : vertexFaceCollisionsPerStrand) {
    vertexFaceCollisions.insert(vertexFaceCollisions.end(), collisions.begin(),
                                collisions.end());
  }

  comb->Draw(vertexFaceCollisions);
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
  comb = std::make_shared<Comb>();
  comb->Translate(Vec3<Real>(-1, -1, 10));
  combTranslation = Vec<Real>::LinSpaced(200, -8, -1).reverse();

  for (Real ss = 11; ss < 12; ss += 0.25) {
    // Construct a trivial point set
    std::vector<Vec3<Real>> points;
    for (Real ii = 0; ii < 8; ii += 0.25) {
      points.emplace_back(ii, 0, ss);
    }

    Mat<Real> v(points.size(), 3);
    for (int ii = 0; ii < points.size(); ++ii) {
      v.row(ii) = points.at(ii);
    }

    auto mesh = std::make_shared<StrandMesh>(v);
    auto integrator =
        std::make_unique<ForwardEulerStrand>(mesh, nullptr, 1.0 / 10'000.0);
    meshes.emplace_back(mesh);
    integrators.emplace_back(std::move(integrator));
  }

  // Zoom out and set the center in a different spot

  //  Radius 15.4999
  //      Theta 0.0108005
  //      Phi 1.5108
  //      Eye 8.77399, -2.67725, 9.03442
  //                    Look At -6.69713, -3.60663, 8.86732
  //                    Up -0.0599567, 0.998201, -0.000647587
  //                 FOV 65
  //

  camera->SetRadius(15.5);
  camera->SetTheta(0.0108005);
  camera->SetPhi(1.5108);

  //  Radius 16.9
  //  Theta 1.4508
  //  Phi 1.7208
  //  Eye 2.00036, -2.52556, 16.59
  //  Look At 0, 0, 0
  //  Up 0.0178895, 0.988771, 0.148367
  //  FOV 65
  camera->SetRadius(16.9);
  camera->SetTheta(1.4508);
  camera->SetPhi(1.7208);
}
