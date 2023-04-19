#pragma once

#include <LibMath.h>

struct BishopFrame {
  // The tangent should always be defined
  Vec3<Real> t = Vec3<Real>::Zero();
  Vec3<Real> u = Vec3<Real>::Zero();
  Vec3<Real> v = Vec3<Real>::Zero();
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
  Vec3<Real> edge;

  // This is the bishop frame
  BishopFrame bishopFrame;

  // This is the "modified" portion, the theta
  Real theta0;
  Real theta1;

  // Constructor which takes only x0 and x1
  ModifiedBishopFrame(const Vec3<Real> &x0, const Vec3<Real> &x1,
                      const Vec3<Real> &t, const Vec3<Real> &u,
                      const Vec3<Real> &v);
  friend auto operator<<(std::ostream &os, const ModifiedBishopFrame &frame)
      -> std::ostream &;
};

class StrandMesh {
public:
  std::vector<ModifiedBishopFrame> rodSegments;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<int> pinned;

  StrandMesh(const Mat<Real> &points);

  void Draw();

private:
  void WalkBishopFrames(const Mat<Real> &points);
};
