#include <TetMesh.h>

namespace sim {

void TetMesh::BuildMassMatrix() {
  m.resize(DOFs(), DOFs());
  m.setZero();

  std::vector<Eigen::Triplet<Real>> mTriplet;
  for (int ii = 0; ii < v.rows(); ++ii) {
    const auto &oneRing = oneRingAreas.at(ii);
    const int index = ii * 3;
    mTriplet.emplace_back(index, index, oneRing);
    mTriplet.emplace_back(index + 1, index + 1, oneRing);
    mTriplet.emplace_back(index + 2, index + 2, oneRing);
  }

  m.setFromTriplets(mTriplet.begin(), mTriplet.end());
}
void TetMesh::ComputeTetRestVolumes() {
  for (int ii = 0; ii < t.rows(); ++ii) {
    restVolumes.at(ii) = TetVolume(v.row(t(ii, 0)), v.row(t(ii, 1)),
                                   v.row(t(ii, 2)), v.row(t(ii, 3)));
    // Prevent negative volumes
    SIM_ASSERT(restVolumes.at(ii) >= 0.0, "");
  }
}
void TetMesh::ComputeVertexAreas() {
  // Fill the vector with zeroes
  oneRingAreas.assign(v.rows(), 0.0);

  for (int ii = 0; ii < t.rows(); ++ii) {
    const Vec4<int> tet = t.row(ii);
    const Vec3<Real> a = v.row(tet(0));
    const Vec3<Real> b = v.row(tet(1));
    const Vec3<Real> c = v.row(tet(2));
    const Vec3<Real> d = v.row(tet(3));
    const Real volume = TetVolume(a, b, c, d);
    oneRingAreas.at(tet(0)) += volume / 4.0;
    oneRingAreas.at(tet(1)) += volume / 4.0;
    oneRingAreas.at(tet(2)) += volume / 4.0;
    oneRingAreas.at(tet(3)) += volume / 4.0;
  }
}
void TetMesh::ComputeDmInverses() {
  for (int ii = 0; ii < t.rows(); ++ii) {
    const Vec4<int> tet = t.row(ii);
    const Vec3<Real> a = rv.row(tet(0));
    const Vec3<Real> b = rv.row(tet(1));
    const Vec3<Real> c = rv.row(tet(2));
    const Vec3<Real> d = rv.row(tet(3));
    Mat<Real> dm(3, 3);
    dm.col(0) = b - a;
    dm.col(1) = c - a;
    dm.col(2) = d - a;
    dmInvs.at(ii) = dm.inverse();
  }
}
void TetMesh::ComputePartialFPartialxs() {
  for (int ii = 0; ii < t.rows(); ++ii) {
    partialFPartialxs.at(ii) = PartialFPartialx(dmInvs[ii]);
  }
}
void TetMesh::InitializeDataStructures() {
  rv = v;
  pinned = Vec<int>::Zero(this->v.rows());

  dmInvs.resize(t.rows());
  partialFPartialxs.resize(t.rows());
  restVolumes.resize(t.rows());
  oneRingAreas.resize(v.rows());

  // Precompute the Dm Inverses
  ComputeDmInverses();

  // Precompute the change-of-basis matrices
  ComputePartialFPartialxs();

  // Compute the tet volumes
  ComputeTetRestVolumes();

  // Compute the one-ring areas
  ComputeVertexAreas();

  // Construct the mass matrix
  BuildMassMatrix();
}
void TetMesh::Tetrahedralize() {
  static const std::string switches = "zpQ";

  igl::copyleft::tetgen::tetrahedralize(Mat<Real>(v), Mat<int>(f), switches, v,
                                        t, f);
  igl::boundary_facets(t, f);

  // The faces come out of this function in the wrong winding order. So
  // this fixes that.
  f.rowwise().reverseInPlace();
}
auto TetMesh::EvalPartialFPartialx(int index, const Mat3<Real> &dmInv)
    -> Mat3<Real> {
  SIM_ASSERT(index >= 0 && index <= 11, "Index: " + std::to_string(index) +
                                        " invalid. Must be in range[0, 11].");

  Mat3<Real> ret = Mat3<Real>::Zero();
  switch (index) {
  case 0: // pfpx0x
    ret.row(0) << -1, -1, -1;
    break;
  case 1: // pfpx0y
    ret.row(1) << -1, -1, -1;
    break;
  case 2: // pfpx0z
    ret.row(2) << -1, -1, -1;
    break;
  case 3: // pfpx1x
    ret(0, 0) = 1;
    break;
  case 4: // pfpx1y
    ret(1, 0) = 1;
    break;
  case 5: // pfpx1z
    ret(2, 0) = 1;
    break;
  case 6: // pfpx2x
    ret(0, 1) = 1;
    break;
  case 7: // pfpx2y
    ret(1, 1) = 1;
    break;
  case 8: // pfpx2z
    ret(2, 1) = 1;
    break;
  case 9: // pfpx3x
    ret(0, 2) = 1;
    break;
  case 10: // pfpx3y
    ret(1, 2) = 1;
    break;
  case 11: // pfpx3z
    ret(2, 2) = 1;
    break;
  default:
    SIM_ASSERT(false, "Unreachable");
  }
  return ret * dmInv;
}
auto TetMesh::PartialFPartialx(const Mat3<Real> &dmInv) -> Mat9x12<Real> {
  Mat9x12<Real> ret = Mat9x12<Real>::Zero();
  for (int ii = 0; ii < 12; ++ii) {
    ret.col(ii) = ColwiseFlatten<Real>(EvalPartialFPartialx(ii, dmInv));
  }
  return ret;
}
auto TetMesh::TetVolume(const Vec3<Real> &a, const Vec3<Real> &b,
                        const Vec3<Real> &c, const Vec3<Real> &d) -> Real {
  const auto d1 = b - a;
  const auto d2 = c - a;
  const auto d3 = d - a;
  return d3.dot(d1.cross(d2)) / 6.0;
}

} // namespace sim
