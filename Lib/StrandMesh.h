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
struct Rod {
  Mat<Real> v;
};

class StrandMesh {
  std::vector<Rod> rods;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<int> pinned;

  void Draw();

  auto CurvatureBinormal(const Vec3<Real> &e0, const Vec3<Real> &e1)
      -> Vec3<Real>;

  auto MaterialCurvature(const Vec3<Real> &curvatureBinormal,
                         const MaterialFrame &frame) -> Vec2<Real>;
};
