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

void DrawText(const std::string &text, int windowWidth, int windowHeight) {
  // Set the current matrix mode to "projection"
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  // Set the viewport to cover the entire screen
  glViewport(0, 0, windowWidth, windowHeight);
  // Set the orthographic projection
  glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
  // Set the current matrix mode to "modelview"
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  // Translate to the bottom right corner of the screen
  int string_width = glutBitmapLength(GLUT_BITMAP_HELVETICA_18,
                                      (const unsigned char *)text.c_str());
  glTranslatef(windowWidth - (string_width + 10), 10, 0);
  // Set the raster position to the bottom left corner of the screen
  glRasterPos2i(0, 0);
  // Loop through the characters in the string and draw them using
  // glutBitmapCharacter
  glColor3f(1, 1, 1);
  for (const auto &c : text) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
  }
  // Pop the modelview matrix
  glPopMatrix();
  // Pop the projection matrix
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  // Set the current matrix mode back to "modelview"
  glMatrixMode(GL_MODELVIEW);
}

void DrawGLGrid(int size, float spacing) {
  glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_LINES);
  for (float ii = -size; ii < size; ii += spacing) {
    glVertex3f(ii, 0, -size);
    glVertex3f(ii, 0, size);

    glVertex3f(size, 0, ii);
    glVertex3f(-size, 0, ii);
  }
  glEnd();
}

void DrawCenterAxes() {
  glBegin(GL_LINES);
  // Red - X Axis
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);

  // Green - Y Axis
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);

  // Blue - Z Axis
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  glEnd();
}
