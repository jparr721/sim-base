#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>

class StrandMesh {
public:
  bool drawMaterialFrame = false;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of der->vertices.
  Vec<int> pinned;

  std::shared_ptr<DiscreteElasticRod> der;

  explicit StrandMesh(const Mat<Real> &points);

  [[nodiscard]] INLINE auto DOFs() const -> int { return der->DOFs(); }
  INLINE void SetPositions(const Vec<Real> &x) { der->SetPositions(x); }
  INLINE auto Positions() const -> Vec<Real> { return der->Positions(); }
  void Draw();

  /**
   * Compute forces on the centerline
   * @return n*3 x 1 vector of forces where n is the number of points
   */
  auto ComputeMaterialForces(bool straight = true) -> Vec<Real>;
};