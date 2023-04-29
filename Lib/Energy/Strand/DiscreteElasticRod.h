#pragma once

#include <Energy/Spring/MassSpring.h>
#include <LibMath.h>

constexpr Real gBendingModulus = 0.001;
constexpr Real gTwistingModulus = 0.07;

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

  std::unique_ptr<MassSpring> massSpring;

  Mat<Real> vertices;
  Mat<Real> restVertices;

  std::vector<Frame> frames;

  std::vector<Vec3<Real>> m1s;
  std::vector<Vec3<Real>> m2s;

  // Holonomy parameters
  // \nabla_{i}\psi_i
  std::vector<Vec3<Real>> holonomyGradients;

  std::vector<Vec3<Real>> kbs;

  // \nabla_{i}(\kappa{b}_)i
  std::vector<Mat3<Real>> kbGradients;

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