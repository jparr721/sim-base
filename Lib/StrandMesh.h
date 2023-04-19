#pragma once

#include <LibMath.h>

struct MaterialFrame {
  // Normal
  Vec3<Real> m1;

  // Binormal
  Vec3<Real> m2;

  // Tangent
  Vec3<Real> tangent;
};

/**
 * Rods make up the strands. Each rod is a collection of vertices. We assume
 * that edges are just vertex[i] to vertex[i+1]. The number of vertices must
 * always be at least 2.
 */
struct ModifiedBishopFrame {
  // These are the values controlling the bishop frame
  Vec3<Real> x0;
  Vec3<Real> x1;
  Vec3<Real> e0;

  // This is the bishop frame
  Vec3<Real> t0;
  Vec3<Real> u;
  Vec3<Real> v;

  // This is the "modified" portion, the theta
  Real theta0;
  Real theta1;

  // Constructor which takes only x0 and x1
  ModifiedBishopFrame(const Vec3<Real> &x0, const Vec3<Real> &x1);
};

class StrandMesh {
  std::vector<ModifiedBishopFrame> rodSegments;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<int> pinned;

  StrandMesh(const Mat3<Real> &points);

  void Draw();

  auto CurvatureBinormal(const Vec3<Real> &e0, const Vec3<Real> &e1)
      -> Vec3<Real>;

  auto MaterialCurvature(const Vec3<Real> &curvatureBinormal,
                         const MaterialFrame &frame) -> Vec2<Real>;
};
