#include <Solid/ElasticSystem.h>

namespace sim {

namespace {

// DOF of coordinate `axis` of corner `corner` of `tet`.
auto Dof(const Mat<int> &t, int tet, int corner, int axis) -> int {
  return 3 * t(tet, corner) + axis;
}

} // namespace

ElasticSystem::ElasticSystem(const TetMesh &mesh, const Material &material)
    : mesh(mesh), material(material), constraints{.dofs = mesh.DOFs()} {
  for (int ii = 0; ii < mesh.pinned.size(); ++ii) {
    if (mesh.pinned(ii) == 1) {
      constraints.pinned.push_back(ii);
    }
  }
}

// F = Ds Dm^-1, with Ds the deformed edge vectors from corner 0
// (Dynamic Deformables, "Computing the Deformation Gradient"). The product is
// dynamic-size on purpose: Eigen's fixed-size 3x3 product rounds differently
// and moves the bunny-drop trajectory off its recorded bits.
auto ElasticSystem::DeformationGradient(const Vec<Real> &x, int tet) const
    -> Mat3<Real> {
  const Vec3<Real> a = x.segment<3>(3 * mesh.t(tet, 0));
  Mat<Real> ds(3, 3);
  for (int corner = 1; corner < 4; ++corner) {
    ds.col(corner - 1) = x.segment<3>(3 * mesh.t(tet, corner)) - a;
  }
  return ds * mesh.DmInverses()[tet];
}

// The force on a tet's corners is -V vec(dF/dx)^T vec(P(F))
// (Dynamic Deformables, "Force Computation").
auto ElasticSystem::Forces(const Vec<Real> &x, const Vec<Real> & /*v*/) const
    -> Vec<Real> {
  Vec<Real> forces = Vec<Real>::Zero(Dofs());
  std::visit(
      [&](const auto &energy) {
        for (int tt = 0; tt < mesh.t.rows(); ++tt) {
          const Mat3<Real> p = energy.Pk1(DeformationGradient(x, tt));
          const Vec12<Real> force = -mesh.RestVolumes()[tt] *
                                    (mesh.PartialFPartialxs()[tt].transpose() *
                                     Vectorize(p));
          for (int corner = 0; corner < 4; ++corner) {
            for (int axis = 0; axis < 3; ++axis) {
              forces(Dof(mesh.t, tt, corner, axis)) += force(3 * corner + axis);
            }
          }
        }
      },
      material);
  return forces;
}

// Each tet adds V (dF/dx)^T H (dF/dx), a 12x12 block, where H is the energy's
// Hessian over vec(F) (Dynamic Deformables, "Hessian Computation"). Clamping H
// per element keeps every block, and so their sum, PSD.
auto ElasticSystem::Stiffness(const Vec<Real> &x, bool clamped) const
    -> SparseMat<Real> {
  std::vector<Eigen::Triplet<Real>> triplets;
  triplets.reserve(144 * mesh.t.rows());
  std::visit(
      [&](const auto &energy) {
        for (int tt = 0; tt < mesh.t.rows(); ++tt) {
          const Mat3<Real> f = DeformationGradient(x, tt);
          const Mat9<Real> h = clamped
                                   ? energy.ClampedHessian(f, RotationVariantSvd(f))
                                   : energy.Hessian(f);
          const Mat9x12<Real> &dfdx = mesh.PartialFPartialxs()[tt];
          const Mat12<Real> block =
              mesh.RestVolumes()[tt] * (dfdx.transpose() * h * dfdx);
          for (int row = 0; row < 12; ++row) {
            for (int col = 0; col < 12; ++col) {
              triplets.emplace_back(Dof(mesh.t, tt, row / 3, row % 3),
                                    Dof(mesh.t, tt, col / 3, col % 3),
                                    block(row, col));
            }
          }
        }
      },
      material);
  SparseMat<Real> k(Dofs(), Dofs());
  k.setFromTriplets(triplets.begin(), triplets.end());
  return k;
}

auto ElasticSystem::DampingMatrix(const Vec<Real> & /*x*/) const
    -> SparseMat<Real> {
  return SparseMat<Real>(Dofs(), Dofs());
}

} // namespace sim
