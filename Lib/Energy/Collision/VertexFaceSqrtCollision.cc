#include "VertexFaceSqrtCollision.h"

#include <vector>

namespace sim {

namespace {

// Below this |t| the direction to the face is undefined, and every evaluation
// returns zero.
constexpr Real kInverseEps = 1e-8;

// v[0] is the colliding vertex and v[1..3] the triangle. e[1] = v[0] - v[2] is
// the vertex relative to the triangle, and e[2] x e[0] its normal.
void VerticesAndEdges(const VecN<12> &x, std::vector<Vec3<Real>> &v,
                      std::vector<Vec3<Real>> &e) {
  v.resize(4);
  for (int i = 0; i < 4; i++) {
    v[i][0] = x[i * 3];
    v[i][1] = x[i * 3 + 1];
    v[i][2] = x[i * 3 + 2];
  }

  e.resize(3);
  e[0] = v[3] - v[2];
  e[1] = v[0] - v[2];
  e[2] = v[1] - v[2];
}

// True when the vertex is behind the triangle's normal.
auto ShouldReverse(const std::vector<Vec3<Real>> &e) -> bool {
  Vec3<Real> n = e[2].cross(e[0]);
  n = n / n.norm();
  return n.dot(e[1]) < 0;
}

// Barycentric coordinates of the closest point on the triangle v[1..3] to
// v[0]. Outside the triangle that point is on the nearest edge, clamped to its
// ends, so the coordinates off that edge are exactly zero.
auto InsideBarycentricCoordinates(const std::vector<Vec3<Real>> &vertices)
    -> Vec3<Real> {
  Vec3<Real> barycentric = GetBarycentricCoordinates(vertices);
  if (barycentric[0] >= 0.0 && barycentric[1] >= 0.0 && barycentric[2] >= 0.0) {
    return barycentric;
  }

  const Real distance12 =
      PointLineDistance(vertices[0], vertices[1], vertices[2]);
  const Real distance23 =
      PointLineDistance(vertices[0], vertices[2], vertices[3]);
  const Real distance31 =
      PointLineDistance(vertices[0], vertices[3], vertices[1]);

  // Less than or equal, so a tie still falls into exactly one branch.
  if (distance12 <= distance23 && distance12 <= distance31) {
    const Vec2<Real> lerp = GetLerp(vertices[0], vertices[1], vertices[2]);
    return {lerp[0], lerp[1], 0.0};
  }
  if (distance23 <= distance12 && distance23 <= distance31) {
    const Vec2<Real> lerp = GetLerp(vertices[0], vertices[2], vertices[3]);
    return {0.0, lerp[0], lerp[1]};
  }
  const Vec2<Real> lerp = GetLerp(vertices[0], vertices[3], vertices[1]);
  return {lerp[1], 0.0, lerp[0]};
}

// dt/dx for t = v0 - sum_i b_i v_{i+1} with b held fixed.
auto TPartial(const Vec3<Real> &bary) -> Mat3x12<Real> {
  Mat3x12<Real> tPartial;
  tPartial.setZero();
  tPartial(0, 0) = tPartial(1, 1) = tPartial(2, 2) = 1.0;
  tPartial(0, 3) = tPartial(1, 4) = tPartial(2, 5) = -bary[0];
  tPartial(0, 6) = tPartial(1, 7) = tPartial(2, 8) = -bary[1];
  tPartial(0, 9) = tPartial(1, 10) = tPartial(2, 11) = -bary[2];
  return tPartial;
}

// Everything one evaluation needs: the closest point's coordinates, the
// vector t from it to the vertex, |t| and the spring term d - sigma eps.
struct Contact {
  std::vector<Vec3<Real>> v;
  Vec3<Real> bary;
  Vec3<Real> t;
  Real tMagnitude;
  Real springDiff;
  bool degenerate;
};

auto Evaluate(const VecN<12> &x, Real epsilon) -> Contact {
  Contact c;
  std::vector<Vec3<Real>> e;
  VerticesAndEdges(x, c.v, e);
  c.bary = InsideBarycentricCoordinates(c.v);
  const bool reversal = ShouldReverse(e);
  const Vec3<Real> xs =
      c.bary[0] * c.v[1] + c.bary[1] * c.v[2] + c.bary[2] * c.v[3];
  c.t = c.v[0] - xs;
  c.tMagnitude = sqrt(c.t.dot(c.t));
  c.springDiff = reversal ? c.tMagnitude + epsilon : c.tMagnitude - epsilon;
  c.degenerate =
      fabs(c.tMagnitude) <= kInverseEps || fabs(c.t.dot(c.t)) < kInverseEps;
  return c;
}

// The Hessian of d = min_b |t(x, b)|. With the active coordinates written as
// b = b0 + B u (u free), the implicit function theorem on dd/du = 0 gives
// d_xx - d_xu d_uu^-1 d_ux, every term taken at fixed b.
auto DistanceHessian(const Contact &c) -> MatN<12> {
  const Vec3<Real> tHat = c.t / c.tMagnitude;
  const Mat3<Real> p = (Mat3<Real>::Identity() - tHat * tHat.transpose()) /
                       c.tMagnitude;
  const Mat3x12<Real> tDiff = TPartial(c.bary);
  const MatN<12> dxx = tDiff.transpose() * p * tDiff;

  // The coordinates the closest point can move along: every nonzero one, less
  // one for the sum-to-one constraint.
  std::vector<int> active;
  for (int i = 0; i < 3; ++i) {
    if (c.bary[i] != 0.0) {
      active.push_back(i);
    }
  }
  const int free = static_cast<int>(active.size()) - 1;
  if (free == 0) {
    return dxx;
  }

  // Column j moves weight from triangle vertex active[0] to active[j + 1].
  Mat<Real> dtdu(3, free);
  Mat<Real> dxu(12, free);
  for (int j = 0; j < free; ++j) {
    const int to = active[j + 1];
    const int from = active[0];
    dtdu.col(j) = c.v[from + 1] - c.v[to + 1];
    Vec<Real> dDdu = Vec<Real>::Zero(12);
    dDdu.segment<3>(3 * (to + 1)) = -tHat;
    dDdu.segment<3>(3 * (from + 1)) = tHat;
    dxu.col(j) = dDdu + tDiff.transpose() * p * dtdu.col(j);
  }
  const Mat<Real> duu = dtdu.transpose() * p * dtdu;
  return dxx - dxu * duu.ldlt().solve(dxu.transpose());
}

} // namespace

VertexFaceSqrtCollision::VertexFaceSqrtCollision(Real collisionStiffness,
                                                 Real collisionEpsilon)
    : collisionStiffness(collisionStiffness),
      collisionEpsilon(collisionEpsilon) {}

auto VertexFaceSqrtCollision::Psi(const VecN<12> &x) const -> Real {
  const Contact c = Evaluate(x, collisionEpsilon);
  return collisionStiffness * c.springDiff * c.springDiff;
}

// 2 k (d - sigma eps) dd/dx, with dd/dx = (dt/dx)^T t / |t| at fixed b.
auto VertexFaceSqrtCollision::Gradient(const VecN<12> &x) const -> VecN<12> {
  const Contact c = Evaluate(x, collisionEpsilon);
  if (c.degenerate) {
    return VecN<12>::Zero();
  }
  const Mat3x12<Real> tDiff = TPartial(c.bary);
  return 2.0 * collisionStiffness * c.springDiff * (1.0 / c.tMagnitude) *
         tDiff.transpose() * c.t;
}

// 2 k (dd/dx dd/dx^T + (d - sigma eps) d^2d/dx^2).
auto VertexFaceSqrtCollision::Hessian(const VecN<12> &x) const -> MatN<12> {
  const Contact c = Evaluate(x, collisionEpsilon);
  if (c.degenerate) {
    return MatN<12>::Zero();
  }
  const VecN<12> dd = TPartial(c.bary).transpose() * (c.t / c.tMagnitude);
  return 2.0 * collisionStiffness *
         (dd * dd.transpose() + c.springDiff * DistanceHessian(c));
}

auto VertexFaceSqrtCollision::ClampedHessian(const VecN<12> &x) const
    -> MatN<12> {
  const Eigen::SelfAdjointEigenSolver<MatN<12>> eigen(Hessian(x));
  const VecN<12> clamped = eigen.eigenvalues().cwiseMax(0);
  return eigen.eigenvectors() * clamped.asDiagonal() *
         eigen.eigenvectors().transpose();
}

} // namespace sim
