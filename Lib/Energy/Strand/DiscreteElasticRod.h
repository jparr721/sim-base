#pragma once

#include <LibMath.h>

constexpr Real gBendingModulus = 1.0;
constexpr Real gTwistingModulus = 0.075;

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

struct Bend {
  Vec2<Real> prevCurvature;
  Vec2<Real> nextCurvature;
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

  // Keeps track of material curvature
  //  std::vector<Bend> bends;
  //  std::vector<Bend> restBends;

  std::vector<Vec2<Real>> curvature;
  std::vector<Vec2<Real>> restCurvature;

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
  INLINE auto Positions() const -> Vec<Real> {
    return RowwiseFlatten(vertices);
  }

  void Initialize();
  auto ComputeCenterlineForcesGeneral() -> Vec<Real>;
  auto ComputeCenterlineForcesStraight() -> Vec<Real>;

  /**
   * Update the bishop frame with a rotation matrix
   */
  void UpdateBishopFrames();
  void UpdateMaterialFrames();
  void UpdateMaterialCurvatures();
  void UpdateQuasistaticMaterialFrame();
  void UpdateKbGradients();
  void UpdateHolonomyGradient();
  void UpdateLengths();

  void Computekbs();

  auto ComputeOmega(const Vec3<Real> &kb, const Vec3<Real> &m1,
                    const Vec3<Real> &m2) -> Vec2<Real>;
  auto ComputeGradOmega(const Mat3<Real> &gradkb, const Vec3<Real> &m1,
                        const Vec3<Real> &m2, const Vec3<Real> &gradpsi,
                        const Vec2<Real> &w) -> Mat2x3<Real>;

  /**
   * Partial of E wrt theta^j from equation 7.
   * @return
   */
  auto ComputeTwistingForce() -> Vec<Real>;
  void ComputeTwistingHessian(Vec<Real> &upper, Vec<Real> &center,
                              Vec<Real> &lower);

  auto ComputepEpxi(int j) -> Vec3<Real>;
};