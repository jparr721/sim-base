#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>

namespace sim {

class StrandMesh {
public:
  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of der->vertices.
  Vec<int> pinned;

  std::shared_ptr<DiscreteElasticRod> der;

  explicit StrandMesh(const Mat<Real> &points);

  [[nodiscard]] SIM_INLINE auto DOFs() const -> int { return der->DOFs(); }
  SIM_INLINE void SetPositions(const Vec<Real> &x) { der->SetPositions(x); }
  SIM_INLINE auto Positions() const -> Vec<Real> { return der->Positions(); }
  SIM_INLINE void TranslatePinnedX(Real dx, int ii) {
    if (pinned[ii] == 1) {
      der->vertices(ii, 0) = der->restVertices(ii, 0) + dx;
    }
  }

  /**
   * Compute forces on the centerline
   * @return n*3 x 1 vector of forces where n is the number of points
   */
  auto ComputeMaterialForces() -> Vec<Real>;
};

} // namespace sim
