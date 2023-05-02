#include "StrandScene.h"
#include <LibGui.h>
#include <igl/parallel_for.h>
#include <igl/readOBJ.h>

Comb::Comb() {
  igl::readOBJ(Meshes / "comb.obj", v, f);
  rv = v;
}

void Comb::Draw() {
  GLOBAL GLfloat lightAmbient[] = {0.2f, 0.2f, 0.2f, 1.0f};
  GLOBAL GLfloat lightDiffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
  GLOBAL GLfloat lightSpecular[] = {0.5f, 0.5f, 0.5f, 1.0f};
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

  // USE THESE TO SHADE MATERIALS
  glShadeModel(GL_SMOOTH);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, gDiffuseGray.data());
  glMaterialfv(GL_FRONT, GL_AMBIENT, gAmbientFull.data());
  glMaterialfv(GL_FRONT, GL_SPECULAR, gSpecularFull.data());
  glMaterialfv(GL_FRONT, GL_SHININESS, gShininess.data());

  glEnable(GL_DEPTH_TEST);

  // Draw lines
  glEnable(GL_POLYGON_OFFSET_LINE);
  glPolygonOffset(-1.0, -1.0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glColor3d(0.0, 0.0, 0.0);
  glBegin(GL_TRIANGLES);
  for (int ii = 0; ii < f.rows(); ++ii) {
    const Vec3<int> face = f.row(ii);
    const Vec3<Real> a = v.row(face(0));
    const Vec3<Real> b = v.row(face(1));
    const Vec3<Real> c = v.row(face(2));
    glVertex3dv(a.data());
    glVertex3dv(b.data());
    glVertex3dv(c.data());
  }
  glEnd();

  // Draw without lines
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glColor4d(0.7, 0.7, 0.7, 0.25);
  glBegin(GL_TRIANGLES);
  for (int ii = 0; ii < f.rows(); ++ii) {
    const Vec3<int> face = f.row(ii);
    const Vec3<Real> a = v.row(face(0));
    const Vec3<Real> b = v.row(face(1));
    const Vec3<Real> c = v.row(face(2));
    glVertex3dv(a.data());
    glVertex3dv(b.data());
    glVertex3dv(c.data());
  }
  glEnd();
  glDisable(GL_POLYGON_OFFSET_LINE);
}

void Comb::Translate(const Vec3<Real> &translation) {
  for (int ii = 0; ii < v.rows(); ++ii) {
    const Vec3<Real> restRow = rv.row(ii);
    v.row(ii) = restRow + translation;
  }
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
  comb->Draw();
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
  comb = std::make_unique<Comb>();
  comb->Translate(Vec3<Real>(-1, -1, 10));
  combTranslation = Vec<Real>::LinSpaced(200, -8, -1).reverse();

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
