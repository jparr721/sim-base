#pragma once

#include "AdaptedFrame.h"
#include <Energy/Spring/MassSpring.h>
#include <LibMath.h>

constexpr Real gBendingModulus = 0.1;
constexpr Real gTwistingModulus = 1;

class DiscreteElasticRod {
public:
  int nRods;
  Real totalRestLength;
  Real rightAngularVelocity = 0.0;
  Real leftAngularVelocity = 0.0;

  std::unique_ptr<MassSpring> massSpring;

  Mat<Real> vertices;
  Mat<Real> restVertices;

  std::vector<Edge> edges;
  std::vector<Edge> restEdges;
  std::vector<BishopFrame> frames;

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

  Vec<Real> thetas;

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
  void UpdateKbGradients();
  void UpdateHolonomyGradient();
  void UpdateLengths();

  void Computekbs();

  auto ComputeOmega(const Vec3<Real> &kb, const Vec3<Real> &m1,
                    const Vec3<Real> &m2) -> Vec2<Real>;
};