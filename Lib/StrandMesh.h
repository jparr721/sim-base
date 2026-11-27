#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>

#include <memory>

namespace sim {

// A strand's live state, as renderers read it.
class StrandMesh {
public:
  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of der->vertices.
  Vec<int> pinned;

  std::shared_ptr<DiscreteElasticRod> der;

  explicit StrandMesh(const Mat<Real> &points);
};

} // namespace sim
