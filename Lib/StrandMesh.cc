#include <StrandMesh.h>

namespace sim {

StrandMesh::StrandMesh(const Mat<Real> &points)
    : pinned(Vec<int>::Zero(points.rows())),
      der(std::make_shared<DiscreteElasticRod>(points)) {}

} // namespace sim
