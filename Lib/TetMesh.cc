#include <TetMesh.h>

void TetMesh::Draw() {
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

  // Draw normals
  if (drawNormals) {
    glColor3d(1.0, 0.0, 1.0);
    glBegin(GL_LINES);
    for (int ii = 0; ii < f.rows(); ++ii) {
      const Vec3<int> face = f.row(ii);
      const Vec3<Real> a = v.row(face(0));
      const Vec3<Real> b = v.row(face(1));
      const Vec3<Real> c = v.row(face(2));

      const Vec3<Real> an = n.row(face(0));
      const Vec3<Real> bn = n.row(face(1));
      const Vec3<Real> cn = n.row(face(2));

      glVertex3dv(a.data());
      glVertex3dv((a + an).eval().data());

      glVertex3dv(b.data());
      glVertex3dv((b + bn).eval().data());

      glVertex3dv(c.data());
      glVertex3dv((c + cn).eval().data());
    }
    glEnd();
  }

  if (drawPinned) {
    glColor3d(1.0, 0.0, 0.0);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (int ii = 0; ii < pinned.size(); ++ii) {
      if (pinned(ii) == 1) {
        const Vec3<Real> a = v.row(ii);
        glVertex3dv(a.data());
      }
    }
    glEnd();
  }
}
void TetMesh::BuildMassMatrix() {
  m.resize(DOFs(), DOFs());
  mInv.resize(DOFs(), DOFs());
  m.setZero();
  mInv.setZero();

  std::vector<Eigen::Triplet<Real>> mTriplet;
  std::vector<Eigen::Triplet<Real>> mInvTriplet;
  for (int ii = 0; ii < v.rows(); ++ii) {
    const auto &oneRing = oneRingAreas.at(ii);
    const int index = ii * 3;
    mTriplet.emplace_back(index, index, oneRing);
    mTriplet.emplace_back(index + 1, index + 1, oneRing);
    mTriplet.emplace_back(index + 2, index + 2, oneRing);
    mInvTriplet.emplace_back(index, index, 1.0 / oneRing);
    mInvTriplet.emplace_back(index + 1, index + 1, 1.0 / oneRing);
    mInvTriplet.emplace_back(index + 2, index + 2, 1.0 / oneRing);
  }

  m.setFromTriplets(mTriplet.begin(), mTriplet.end());
  mInv.setFromTriplets(mInvTriplet.begin(), mInvTriplet.end());
}
void TetMesh::ComputeTetRestVolumes() {
  for (int ii = 0; ii < t.rows(); ++ii) {
    restVolumes.at(ii) = TetVolume(v.row(t(ii, 0)), v.row(t(ii, 1)),
                                   v.row(t(ii, 2)), v.row(t(ii, 3)));
    // Prevent negative volumes
    ASSERT2(restVolumes.at(ii) >= 0.0);
  }
}
void TetMesh::ComputeVertexAreas() {
  // Fill the vector with zeroes
  oneRingAreas.assign(t.rows(), 0.0);

  for (int ii = 0; ii < t.rows(); ++ii) {
    const Vec4<int> tet = t.row(ii);
    const Vec3<Real> a = v.row(tet(0));
    const Vec3<Real> b = v.row(tet(1));
    const Vec3<Real> c = v.row(tet(2));
    const Vec3<Real> d = v.row(tet(3));
    const Real volume = TetVolume(a, b, c, d);
    oneRingAreas.at(tet(0)) += volume / 4.0;
    oneRingAreas.at(tet(1)) += volume / 4.0;
    oneRingAreas.at(tet(2)) += volume / 4.0;
    oneRingAreas.at(tet(3)) += volume / 4.0;
  }
}
void TetMesh::ComputeDmInverses() {
  for (int ii = 0; ii < t.rows(); ++ii) {
    const Vec4<int> tet = t.row(ii);
    const Vec3<Real> a = rv.row(tet(0));
    const Vec3<Real> b = rv.row(tet(1));
    const Vec3<Real> c = rv.row(tet(2));
    const Vec3<Real> d = rv.row(tet(3));
    Mat<Real> dm(3, 3);
    dm.col(0) = b - a;
    dm.col(1) = c - a;
    dm.col(2) = d - a;
    dmInvs.at(ii) = dm.inverse();
  }
}
void TetMesh::ComputePartialFPartialxs() {
  for (int ii = 0; ii < t.rows(); ++ii) {
    partialFPartialxs.at(ii) = PartialFPartialx(dmInvs[ii]);
  }
}
void TetMesh::InitializeDataStructures() {
  rv = v;
  pinned = Vec<int>::Zero(this->v.rows());

  dmInvs.resize(t.rows());
  partialFPartialxs.resize(t.rows());
  restVolumes.resize(t.rows());
  oneRingAreas.resize(v.rows());
  fs.resize(t.rows());

  // Precompute the Dm Inverses
  ComputeDmInverses();

  // Precompute the change-of-basis matrices
  ComputePartialFPartialxs();

  // Compute the tet volumes
  ComputeTetRestVolumes();

  // Compute the one-ring areas
  ComputeVertexAreas();

  // Construct the mass matrix and inverse mass matrix
  BuildMassMatrix();

  needsNewDeformationGradients = true;
}
void TetMesh::Tetrahedralize() {
  static const std::string switches = "zpQ";

  igl::copyleft::tetgen::tetrahedralize(Mat<Real>(v), Mat<int>(f), switches,
                                        v, t, f);
  igl::boundary_facets(t, f);

  // The faces come out of this function in the wrong winding order. So
  // this fixes that.
  f.rowwise().reverseInPlace();
}
