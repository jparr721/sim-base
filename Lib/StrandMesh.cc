#include <StrandMesh.h>

StrandMesh::StrandMesh(const Mat<Real> &points)
    : pinned(Vec<int>::Zero(points.rows())),
      der(new DiscreteElasticRod(points)) {}

auto StrandMesh::ComputeMaterialForces(bool straight) -> Vec<Real> {
  if (straight) {
    return der->ComputeCenterlineForcesStraight();
  }

  Vec<Real> R = der->ComputeCenterlineForcesGeneral();

  // Add collision forces

  return R;
}
