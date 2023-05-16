#include "StrandDropScene.h"
#include <LibGui.h>
#include <igl/parallel_for.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

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
    integrator->Step(sceneFrames > 0);
  });
}

void StrandDropScene::Draw() {
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

void StrandDropScene::DumpFrame() {
  char filename[512];
  sprintf(filename, "/Users/jarredparr/Downloads/output/frame_%04i.obj", frame);

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

  char combFilename[512];
  sprintf(combFilename, "/Users/jarredparr/Downloads/output/comb_%04i.obj",
          frame);

  igl::writeOBJ(combFilename, comb->v, comb->f);
}

DiscreteElasticRods::DiscreteElasticRods(
    std::shared_ptr<Camera<float>> &camera) {
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
    for (int ii = 0; ii < points.size(); ++ii) {
      v.row(ii) = points.at(ii);
    }

    auto mesh = std::make_shared<StrandMesh>(v);
    mesh->pinned(0) = 1;
    auto integrator = std::make_unique<ForwardEulerStrand>(
        mesh, nullptr, 1.0 / 1'000.0, 5, 0.3);
    meshes.emplace_back(mesh);
    integrators.emplace_back(std::move(integrator));
  }

  // Must set eye, lookat, and up and then displace
  camera->SetEye(Vec3<float>(12.4994, 4.6141, 10.0808));
  camera->SetLookAt(Vec3<float>(-0.367183, -0.0835798, 9.81318));
  camera->SetUp(Vec3<float>(-0.342824, 0.939373, -0.00713152));
  camera->SetDisplacement(Vec3<float>(-0.367183, -0.0835798, 9.81318));
}
