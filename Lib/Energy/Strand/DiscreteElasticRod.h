#pragma once

#include <LibMath.h>

INLINE auto RotationMatrixAroundNormal(const Vec3<Real> &axisAngle)
    -> Mat3<Real> {
  double angle = axisAngle.norm(); // Get the angle of rotation from the norm
                                   // of the axis-angle vector
  Vec3<Real> axis = axisAngle.normalized(); // Normalize the axis of rotation
  Eigen::AngleAxisd rotation(
      angle,
      axis); // Create an angle-axis object with the rotation axis and angle
  Mat3<Real> rotationMatrix =
      rotation.toRotationMatrix(); // Convert the angle-axis object to a
                                   // rotation matrix
  return rotationMatrix;
}

INLINE auto OrthogonalVector(const Vec3<Real> &v) -> Vec3<Real> {
  // Find an arbitrary vector
  Vec3<Real> arbitrary = Vec3<Real>::UnitX();
  if (v.isApprox(Vec3<Real>::UnitX())) {
    arbitrary = Vec3<Real>::UnitY();
  }
  // Compute the cross product between the given vector and the arbitrary vector
  Vec3<Real> result = v.cross(arbitrary);
  // Normalize the result vector
  result.normalize();
  return result;
}

struct Frame {
  // The tangent should always be defined
  Vec3<Real> t = Vec3<Real>::Zero();
  Vec3<Real> u = Vec3<Real>::Zero();
  Vec3<Real> v = Vec3<Real>::Zero();

  Frame() = default;
  Frame(const Vec3<Real> &t, const Vec3<Real> &u, const Vec3<Real> &v)
      : t(t), u(u), v(v) {}
};

class DiscreteElasticRod {
public:
  int nRods;

  Mat<Real> vertices;
  Mat<Real> restVertices;

  std::vector<Frame> frames;
  std::vector<Vec3<Real>> kbs;

  Vec<Real> velocities;
  Vec<Real> thetas;
  Vec<Real> lengths;
  Vec<Real> restLengths;

  DiscreteElasticRod(Mat<Real> vertices);

  void Initialize();

  /**
   * Update the bishop frame with a rotation matrix
   */
  void UpdateBishopFrames();

  void Computekbs();
};