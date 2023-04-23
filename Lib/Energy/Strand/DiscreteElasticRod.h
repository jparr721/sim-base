#pragma once

#include <LibMath.h>

#define BENDING_MODULUS 1.0
#define TWISTING_MODULUS 0.075;

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

  std::vector<Vec3<Real>> m1s;
  std::vector<Vec3<Real>> m2s;

  // Holonomy parameters
  // \nabla_{i-1}\psi_i
  std::vector<Vec3<Real>> holonomyLhs;

  // \nabla_{i+1}\psi_i
  std::vector<Vec3<Real>> holonomyRhs;

  // \nabla_{i}\psi_i
  std::vector<Vec3<Real>> holonomy;

  std::vector<Vec3<Real>> kbs;

  // \nabla_{i-1}(\kappa{b}_)i
  std::vector<Mat3<Real>> gradientLhskbs;

  // \nabla_{i+1}(\kappa{b}_)i
  std::vector<Mat3<Real>> gradientRhskbs;

  // \nabla_{i}(\kappa{b}_)i
  std::vector<Mat3<Real>> gradientkbs;

  std::vector<Vec2<Real>> nextCurvature;
  std::vector<Vec2<Real>> prevCurvature;

  std::vector<Vec2<Real>> restNextCurvature;
  std::vector<Vec2<Real>> restPrevCurvature;

  Vec<Real> velocities;
  Vec<Real> thetas;
  Vec<Real> lengths;
  Vec<Real> restLengths;

  SparseMat<Real> mInv;

  explicit DiscreteElasticRod(Mat<Real> vertices);

  [[nodiscard]] INLINE auto DOFs() const -> int { return 3 * vertices.rows(); }
  INLINE void SetPositions(const Vec<Real> &x) {
    vertices = RowwiseUnFlatten(x, vertices.rows(), vertices.cols());
  }

  void Initialize();
  auto ComputeCenterlineForces() -> Vec<Real>;

  /**
   * Update the bishop frame with a rotation matrix
   */
  void UpdateBishopFrames();
  void UpdateMaterialFrames();
  void UpdateMaterialCurvatures();
  void UpdateKbGradients();
  void UpdateHolonomyGradient();

  void Computekbs();

  auto ComputeW(const Vec3<Real> &kb, const Vec3<Real> &m1,
                const Vec3<Real> &m2) -> Vec2<Real>;
  auto ComputeGradW(const Vec3<Real> &kb, const Mat3<Real> &gradkb,
                    const Vec3<Real> &m1, const Vec3<Real> &m2,
                    const Vec3<Real> &psi, const Vec2<Real> &w) -> Mat2x3<Real>;

  auto ComputePEPTheta(int ii, const Vec3<Real> &m1j, const Vec3<Real> &m2j,
                       const Mat2<Real> &JtimesB) -> Real;
};