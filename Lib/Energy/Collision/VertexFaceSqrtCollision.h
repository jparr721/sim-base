#pragma once

#include <LibMath.h>

class VertexFaceSqrtCollision {
public:
  Real collisionStiffness;
  Real collisionEpsilon;
  VertexFaceSqrtCollision(Real collisionStiffness, Real collisionEpsilon);

  [[nodiscard]] auto Gradient(const Vec12<Real> &x) const -> Vec12<Real>;
  [[nodiscard]] auto
  GetInsideBarycentricCoordinates(const std::vector<Vec3<Real>> &vertices) const
      -> Vec3<Real>;
  [[nodiscard]] auto tDiffPartial(const Vec3<Real> &bary) const
      -> Mat3x12<Real>;

  INLINE void GetVerticesAndEdges(const Vec12<Real> &x,
                                  std::vector<Vec3<Real>> &v,
                                  std::vector<Vec3<Real>> &e) const {
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

  [[nodiscard]] INLINE auto
  ShouldReverse(const std::vector<Vec3<Real>> &v,
                const std::vector<Vec3<Real>> &e) const -> bool {
    // get the normal
    Vec3<Real> n = e[2].cross(e[0]);
    n = n / n.norm();

    // e[1] is already the collision vertex recentered to the origin
    // (v[0] - v[2])
    const Real dotted = n.dot(e[1]);

    return dotted < 0;
  }
};
