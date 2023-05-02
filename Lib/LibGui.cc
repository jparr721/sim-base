#include <LibGui.h>

void DrawSurfaceMesh(const Mat<Real> &v, const Mat<int> &f) {
  GLOBAL GLfloat lightAmbient[] = {0.2f, 0.2f, 0.2f, 1.0f};
  GLOBAL GLfloat lightDiffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
  GLOBAL GLfloat lightSpecular[] = {0.5f, 0.5f, 0.5f, 1.0f};
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

  // USE THESE TO SHADE MATERIALS
  glShadeModel(GL_SMOOTH);

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

void DrawCylinder(const Vec3<Real> &p0, const Vec3<Real> &p1, Real radius) {
  // Get the distance between the two points
  Real length = (p1 - p0).norm();

  // Get the vector pointing from p0 to p1
  Vec3<Real> direction = p1 - p0;
  Vec3<Real> ax = direction.cross(Vec3<Real>::UnitZ().eval());

  // Compute the rotation axis and angle
  Real angle = AngleBetweenVectors(ax.normalized(), Vec3<Real>::UnitZ().eval());

  // Save the current modelview matrix
  glPushMatrix();

  // Translate to the center of the cylinder
  glTranslated(p1.x(), p1.y(), p1.z());

  // Rotate the cylinder
  glRotated(RadiansToDegrees(angle), ax.x(), ax.y(), ax.z());

  // Draw the cylinder
  GLUquadricObj *quadric = gluNewQuadric();
  gluCylinder(quadric, radius, radius, length, 32, 1);
  gluDeleteQuadric(quadric);

  // Restore the modelview matrix
  glPopMatrix();
}
