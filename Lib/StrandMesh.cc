#include <StrandMesh.h>

void StrandMesh::Draw() {}

auto StrandMesh::CurvatureBinormal(const Vec3<Real> &e0, const Vec3<Real> &e1)
    -> Vec3<Real> {
  return 2 * e0.cross(e1) / (e0.dot(e1) + e0.norm() * e1.norm());
}

auto StrandMesh::MaterialCurvature(const Vec3<Real> &curvatureBinormal,
                                   const MaterialFrame &frame) -> Vec2<Real> {
  return {curvatureBinormal.dot(frame.m1), curvatureBinormal.dot(frame.m2)};
}
