#include "VertexFaceSqrtCollision.h"

#include <vector>

namespace sim {

namespace {

// v[0] is the colliding vertex and v[1..3] the triangle.
auto Vertices(const VecN<12> &x) -> std::vector<Vec3<Real>> {
  std::vector<Vec3<Real>> v(4);
  for (int i = 0; i < 4; i++) {
    v[i] = x.segment<3>(3 * i);
  }
  return v;
}

// Barycentric coordinates of the closest point to v[0] on the boundary of the
// triangle v[1..3]: on the nearest edge, clamped to its ends, so the
// coordinate off that edge is exactly zero.
auto BoundaryBarycentricCoordinates(const std::vector<Vec3<Real>> &vertices)
    -> Vec3<Real> {
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

// A scalar of x with its gradient and Hessian, each filled only when asked.
struct Derivatives {
  Real value = 0;
  VecN<12> gradient = VecN<12>::Zero();
  MatN<12> hessian = MatN<12>::Zero();
};

enum class Order { Value, Gradient, Hessian };

// s = n . q, the signed distance from the triangle's plane to the vertex, with
// q = v0 - v2, u = v1 - v2, w = v3 - v2 and n = N / |N|, N = u x w. Over
// y = (q, u, w), ds/dq = n and ds/dN = P q / |N| =: g, P = I - n n^T, and
// dN = -[w]x du + [u]x dw. N is bilinear in (u, w), which adds g . (u x w)'s
// cross term to the Hessian.
auto PlaneDistance(const std::vector<Vec3<Real>> &v, Order order)
    -> Derivatives {
  const Vec3<Real> q = v[0] - v[2];
  const Vec3<Real> u = v[1] - v[2];
  const Vec3<Real> w = v[3] - v[2];
  const Vec3<Real> normal = u.cross(w);
  const Real area = normal.norm();
  const Vec3<Real> n = normal / area;
  Derivatives s{.value = n.dot(q)};
  if (order == Order::Value) {
    return s;
  }

  // y = M x.
  Eigen::Matrix<Real, 9, 12> m = Eigen::Matrix<Real, 9, 12>::Zero();
  for (int block = 0; block < 3; ++block) {
    m.block<3, 3>(3 * block, 6) = -Mat3<Real>::Identity();
  }
  m.block<3, 3>(0, 0) = m.block<3, 3>(3, 3) = m.block<3, 3>(6, 9) =
      Mat3<Real>::Identity();

  Eigen::Matrix<Real, 3, 6> dNduw;
  dNduw << -CrossProductMatrix(w), CrossProductMatrix(u);
  const Vec3<Real> g = (q - s.value * n) / area;
  Vec9<Real> dsdy;
  dsdy << n, dNduw.transpose() * g;
  s.gradient = m.transpose() * dsdy;
  if (order == Order::Gradient) {
    return s;
  }

  const Mat3<Real> p = Mat3<Real>::Identity() - n * n.transpose();
  const Mat3<Real> dgdN =
      (-(q * n.transpose() + n * q.transpose()) -
       s.value * Mat3<Real>::Identity() + 3 * s.value * n * n.transpose()) /
      (area * area);
  Mat9<Real> h = Mat9<Real>::Zero();
  h.block<3, 6>(0, 3) = p / area * dNduw;
  h.block<6, 3>(3, 0) = h.block<3, 6>(0, 3).transpose();
  h.block<6, 6>(3, 3) = dNduw.transpose() * dgdN * dNduw;
  h.block<3, 3>(3, 6) -= CrossProductMatrix(g);
  h.block<3, 3>(6, 3) += CrossProductMatrix(g);
  s.hessian = m.transpose() * h * m;
  return s;
}

// The Hessian of d = min_b |t(x, b)|. With the active coordinates written as
// b = b0 + B u (u free), the implicit function theorem on dd/du = 0 gives
// d_xx - d_xu d_uu^-1 d_ux, every term taken at fixed b.
auto DistanceHessian(const std::vector<Vec3<Real>> &v, const Vec3<Real> &bary,
                     const Vec3<Real> &t) -> MatN<12> {
  const Real d = t.norm();
  const Vec3<Real> tHat = t / d;
  const Mat3<Real> p = (Mat3<Real>::Identity() - tHat * tHat.transpose()) / d;
  const Mat3x12<Real> tDiff = TPartial(bary);
  const MatN<12> dxx = tDiff.transpose() * p * tDiff;

  // The coordinates the closest point can move along: every nonzero one, less
  // one for the sum-to-one constraint.
  std::vector<int> active;
  for (int i = 0; i < 3; ++i) {
    if (bary[i] != 0.0) {
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
    dtdu.col(j) = v[from + 1] - v[to + 1];
    Vec<Real> dDdu = Vec<Real>::Zero(12);
    dDdu.segment<3>(3 * (to + 1)) = -tHat;
    dDdu.segment<3>(3 * (from + 1)) = tHat;
    dxu.col(j) = dDdu + tDiff.transpose() * p * dtdu.col(j);
  }
  const Mat<Real> duu = dtdu.transpose() * p * dtdu;
  return dxx - dxu * duu.ldlt().solve(dxu.transpose());
}

// phi, the distance the energy springs to epsilon. Over the face it is the
// plane distance s. Off the face it is d - 2 m^2 / d, with d the distance to
// the closest point on the triangle's edges and m = min(s, 0): d in front, and
// behind, a blend that meets d on the plane and -|s| = s at the face's edge.
// phi and its gradient are continuous everywhere but on the triangle's
// boundary itself, where d has a cone.
auto Phi(const VecN<12> &x, Order order) -> Derivatives {
  const std::vector<Vec3<Real>> v = Vertices(x);
  const Derivatives s = PlaneDistance(v, order);
  const Vec3<Real> projected = GetBarycentricCoordinates(v);
  if (projected.minCoeff() >= 0) {
    return s;
  }
  const Vec3<Real> bary = BoundaryBarycentricCoordinates(v);
  const Vec3<Real> t =
      v[0] - (bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3]);
  const Real d = t.norm();
  // On the boundary itself, where phi = s = 0.
  if (d == 0) {
    return s;
  }
  const Real m = std::min(s.value, Real(0));
  Derivatives phi{.value = d - 2 * m * m / d};
  if (order == Order::Value) {
    return phi;
  }

  const VecN<12> dd = TPartial(bary).transpose() * (t / d);
  phi.gradient = (1 + 2 * m * m / (d * d)) * dd - (4 * m / d) * s.gradient;
  if (order == Order::Gradient) {
    return phi;
  }

  const VecN<12> &ds = s.gradient;
  phi.hessian = (1 + 2 * m * m / (d * d)) * DistanceHessian(v, bary, t);
  if (m < 0) {
    phi.hessian +=
        (4 * m / (d * d)) * (dd * ds.transpose() + ds * dd.transpose()) -
        (4 * m * m / (d * d * d)) * dd * dd.transpose() -
        (4 / d) * ds * ds.transpose() - (4 * m / d) * s.hessian;
  }
  return phi;
}

} // namespace

VertexFaceSqrtCollision::VertexFaceSqrtCollision(Real collisionStiffness,
                                                 Real collisionEpsilon)
    : collisionStiffness(collisionStiffness),
      collisionEpsilon(collisionEpsilon) {}

auto VertexFaceSqrtCollision::Psi(const VecN<12> &x) const -> Real {
  const Real spring = Phi(x, Order::Value).value - collisionEpsilon;
  return collisionStiffness * spring * spring;
}

auto VertexFaceSqrtCollision::Gradient(const VecN<12> &x) const -> VecN<12> {
  const Derivatives phi = Phi(x, Order::Gradient);
  return 2.0 * collisionStiffness * (phi.value - collisionEpsilon) *
         phi.gradient;
}

auto VertexFaceSqrtCollision::Hessian(const VecN<12> &x) const -> MatN<12> {
  const Derivatives phi = Phi(x, Order::Hessian);
  return 2.0 * collisionStiffness *
         (phi.gradient * phi.gradient.transpose() +
          (phi.value - collisionEpsilon) * phi.hessian);
}

auto VertexFaceSqrtCollision::ClampedHessian(const VecN<12> &x) const
    -> MatN<12> {
  const Eigen::SelfAdjointEigenSolver<MatN<12>> eigen(Hessian(x));
  const VecN<12> clamped = eigen.eigenvalues().cwiseMax(0);
  return eigen.eigenvectors() * clamped.asDiagonal() *
         eigen.eigenvectors().transpose();
}

} // namespace sim
