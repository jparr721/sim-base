#include <StrandMesh.h>

namespace sim {

StrandMesh::StrandMesh(const Mat<Real> &points)
    : pinned(Vec<int>::Zero(points.rows())),
      der(new DiscreteElasticRod(points)) {}

auto StrandMesh::ComputeMaterialForces() -> Vec<Real> {
  return der->ComputeCenterlineForcesStraight();
}

} // namespace sim
