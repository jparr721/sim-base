#include <LibGui.h>

void DrawCylinder(const Vec3<Real> &p0, const Vec3<Real> &p1, Real radius) {
  // Get the distance between the two points
  Real length = (p1 - p0).norm();

  // Get the vector pointing from p0 to p1
  Vec3<Real> direction = p1 - p0;

  // Compute the rotation axis and angle
  Vec3<Real> axis = direction.normalized();
  //  Real angle = std::acos(axis.dot(Vec3<Real>::UnitX()));
  const Real angle = AngleBetweenVectors(axis, Vec3<Real>::UnitX().eval());

  // Compute the translation vector
  Vec3<Real> translation = (p0 + p1) / 2.0;

  // Save the current modelview matrix
  glPushMatrix();

  // Translate to the center of the cylinder
  glTranslated(translation.x(), translation.y(), translation.z());

  // Rotate the cylinder
  glRotated(angle * 180.0 / M_PI, axis.x(), axis.y(), axis.z());

  // Draw the cylinder
  GLUquadricObj *quadric = gluNewQuadric();
  gluCylinder(quadric, radius, radius, length, 32, 1);
  gluDeleteQuadric(quadric);

  // Restore the modelview matrix
  glPopMatrix();
}
