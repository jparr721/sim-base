#pragma once

#include <LibMath.h>

struct Edge {
  Vec3<Real> v1 = Vec3<Real>::Zero();
  Vec3<Real> v2 = Vec3<Real>::Zero();
  Vec3<Real> e = Vec3<Real>::Zero();
  Vec3<Real> t = Vec3<Real>::Zero();
  Real length = 0;

  Edge(const Vec3<Real> &v1, const Vec3<Real> &v2) : v1(v1), v2(v2) {
    e = v2 - v1;
    length = e.norm();
    t = e.normalized();
  }
};

auto ComputeEdges(const Mat<Real> &vertices) -> std::vector<Edge>;

struct BishopFrame {
  Vec3<Real> t = Vec3<Real>::Zero();
  Vec3<Real> u = Vec3<Real>::Zero();
  Vec3<Real> v = Vec3<Real>::Zero();

  BishopFrame() = default;
  BishopFrame(const Vec3<Real> &t, const Vec3<Real> &u, const Vec3<Real> &v)
      : t(t), u(u), v(v) {}
};

void ParallelTransportModifiedBishopFrames(
    const std::vector<Vec3<Real>> &curvatureBinormals,
    const std::vector<Edge> &edges, std::vector<BishopFrame> &frames);

struct AdaptedFrame {
  BishopFrame frame;
  Vec3<Real> m1;
  Vec3<Real> m2;
  Real theta;

  AdaptedFrame(BishopFrame frame);
};
