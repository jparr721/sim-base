#include <LibGui.h>

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
