#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>

class StrandMesh {
public:
  std::shared_ptr<DiscreteElasticRod> der;

  explicit StrandMesh(const Mat<Real> &points);

  [[nodiscard]] INLINE auto DOFs() const -> int { return der->DOFs(); }
  void Draw();
  auto ComputeMaterialForces() -> Vec<Real>;
};