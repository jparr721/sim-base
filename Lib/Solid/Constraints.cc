#include <Solid/Constraints.h>

namespace sim {

auto Constraints::Filter() const -> SparseMat<Real> {
  Vec<Real> diagonal = Vec<Real>::Ones(dofs);
  for (const int vertex : pinned) {
    diagonal.segment<3>(3 * vertex).setZero();
  }
  return ConstructDiagonalSparseMatrix(diagonal);
}

} // namespace sim
