#include <Solid/ElasticSystem.h>

#include <utility>

namespace sim {

namespace {

// DOF of coordinate `axis` of corner `corner` of `tet`.
auto Dof(const Mat<int> &t, int tet, int corner, int axis) -> int {
  return 3 * t(tet, corner) + axis;
}

// The vertex, then the triangle with its winding reversed, since the
// collision energy's front is the side the clockwise normal faces.
auto ContactDofs(const Vec<Real> &x, const SoftContact &c) -> VecN<12> {
  VecN<12> stacked;
  stacked << x.segment<3>(3 * c.vertex), c.triangle[0], c.triangle[2],
      c.triangle[1];
  return stacked;
}

auto UnitNormal(const SoftContact &c) -> Vec3<Real> {
  return (c.triangle[1] - c.triangle[0])
      .cross(c.triangle[2] - c.triangle[0])
      .normalized();
}

auto ClampedToPsd(const Mat3<Real> &h) -> Mat3<Real> {
  const Eigen::SelfAdjointEigenSolver<Mat3<Real>> eigen(h);
  return eigen.eigenvectors() *
         eigen.eigenvalues().cwiseMax(0).asDiagonal() *
         eigen.eigenvectors().transpose();
}

void AddBlock(std::vector<Eigen::Triplet<Real>> &triplets, int vertex,
              const Mat3<Real> &block) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      triplets.emplace_back(3 * vertex + row, 3 * vertex + col,
                            block(row, col));
    }
  }
}

// Adds a tet's 12x12 block over its corners' DOFs.
void AddTetBlock(std::vector<Eigen::Triplet<Real>> &triplets,
                 const Mat<int> &t, int tet, const Mat12<Real> &block) {
  for (int row = 0; row < 12; ++row) {
    for (int col = 0; col < 12; ++col) {
      triplets.emplace_back(Dof(t, tet, row / 3, row % 3),
                            Dof(t, tet, col / 3, col % 3), block(row, col));
    }
  }
}

auto TetValues(const Mat<int> &t, int tet, const Vec<Real> &y) -> Vec12<Real> {
  Vec12<Real> out;
  for (int corner = 0; corner < 4; ++corner) {
    out.segment<3>(3 * corner) = y.segment<3>(3 * t(tet, corner));
  }
  return out;
}

} // namespace

ElasticSystem::ElasticSystem(const TetMesh &mesh, const Material &material,
                             Damping damping, ContactModel contact, Real drag,
                             std::optional<FiberMaterial> fiber)
    : mesh(mesh), material(material), damping(std::move(damping)),
      contact(contact), drag(drag), fiber(std::move(fiber)),
      constraints{.dofs = mesh.DOFs()} {
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

auto ElasticSystem::VertexMass(int vertex) const -> Real {
  return mesh.m.coeff(3 * vertex, 3 * vertex);
}

auto ElasticSystem::ContactEnergy(int vertex) const -> VertexFaceSqrtCollision {
  return {VertexMass(vertex) * contact.stiffness, contact.epsilon};
}

// Sum over tets and densities of V Psi(F), plus each contact's collision
// energy.
auto ElasticSystem::Energy(const Vec<Real> &x) const -> Real {
  Real energy = 0;
  ForEachDensity(
      [&](const auto &density) {
        for (int tt = 0; tt < mesh.t.rows(); ++tt) {
          energy += mesh.RestVolumes()[tt] *
                    density.Psi(DeformationGradient(x, tt));
        }
      });
  for (const SoftContact &c : contacts) {
    energy += ContactEnergy(c.vertex).Psi(ContactDofs(x, c));
  }
  return energy;
}

// The force on a tet's corners is -V vec(dF/dx)^T vec(P(F))
// (Dynamic Deformables, "Force Computation"). A contact pushes its vertex by
// minus the vertex block of the collision gradient and damps its velocity
// along the triangle normal.
auto ElasticSystem::ForcesWithoutMaterialDamping(const Vec<Real> &x,
                                                 const Vec<Real> &v) const
    -> Vec<Real> {
  Vec<Real> forces = Vec<Real>::Zero(Dofs());
  ForEachDensity(
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
      });
  for (const SoftContact &c : contacts) {
    const Vec3<Real> n = UnitNormal(c);
    forces.segment<3>(3 * c.vertex) -=
        ContactEnergy(c.vertex).Gradient(ContactDofs(x, c)).head<3>() +
        VertexMass(c.vertex) * contact.damping *
            n.dot(v.segment<3>(3 * c.vertex)) * n;
  }
  if (drag != 0) {
    forces -= drag * (mesh.m * v);
  }
  return forces;
}

// Adds a damping stress P's force on the tet's corners, -V (dF/dx)^T vec(P).
void ElasticSystem::AddDampingStress(Vec<Real> &forces, int tet,
                                     const Mat3<Real> &p) const {
  const Vec12<Real> force = -mesh.RestVolumes()[tet] *
                            (mesh.PartialFPartialxs()[tet].transpose() *
                             Vectorize(p));
  for (int corner = 0; corner < 4; ++corner) {
    forces.segment<3>(3 * mesh.t(tet, corner)) += force.segment<3>(3 * corner);
  }
}

// A DampingEnergy's stress is P(F, Fdot), with vec(Fdot) = (dF/dx) v.
auto ElasticSystem::Forces(const Vec<Real> &x, const Vec<Real> &v) const
    -> Vec<Real> {
  Vec<Real> forces = ForcesWithoutMaterialDamping(x, v);
  if (v.isZero(0)) {
    return forces;
  }
  if (const auto *green = std::get_if<GreenDamping>(&damping)) {
    for (int tt = 0; tt < mesh.t.rows(); ++tt) {
      const Vec9<Real> fdot =
          mesh.PartialFPartialxs()[tt] * TetValues(mesh.t, tt, v);
      AddDampingStress(forces, tt,
                       green->Pk1(DeformationGradient(x, tt),
                                  Eigen::Map<const Mat3<Real>>(fdot.data())));
    }
  }
  if (const auto *rayleigh = std::get_if<Rayleigh>(&damping)) {
    forces -= RayleighMatrix(*rayleigh, x) * v;
  }
  return forces;
}

// Green damping's stress is its StepPk1 from F(xV) over h = 1 / alphaV, and
// Rayleigh's matrix is the one at xV.
auto ElasticSystem::StepForces(const Vec<Real> &x, const Vec<Real> &xV,
                               Real alphaV) const -> Vec<Real> {
  const Vec<Real> v = alphaV * (x - xV);
  Vec<Real> forces = ForcesWithoutMaterialDamping(x, v);
  if (v.isZero(0)) {
    return forces;
  }
  if (const auto *green = std::get_if<GreenDamping>(&damping)) {
    for (int tt = 0; tt < mesh.t.rows(); ++tt) {
      AddDampingStress(forces, tt,
                       green->StepPk1(DeformationGradient(x, tt),
                                      DeformationGradient(xV, tt), 1 / alphaV));
    }
  }
  if (const auto *rayleigh = std::get_if<Rayleigh>(&damping)) {
    forces -= RayleighMatrix(*rayleigh, xV) * v;
  }
  return forces;
}

// alphaV / 2 |x - xV|^2_D with D the matrix damping at xV, which is
// 1 / alphaV times the Rayleigh dissipation potential v^T D v / 2, plus
// Green damping's StepPsi over each tet.
auto ElasticSystem::Dissipation(const Vec<Real> &x, const Vec<Real> &xV,
                                Real alphaV) const -> Real {
  const Vec<Real> dV = x - xV;
  Real dissipation = alphaV / 2 * dV.dot(MatrixDamping(xV) * dV);
  const auto *green = std::get_if<GreenDamping>(&damping);
  if (green != nullptr && alphaV != 0) {
    for (int tt = 0; tt < mesh.t.rows(); ++tt) {
      dissipation += mesh.RestVolumes()[tt] *
                     green->StepPsi(DeformationGradient(x, tt),
                                    DeformationGradient(xV, tt), 1 / alphaV);
    }
  }
  return dissipation;
}

// Each tet adds V (dF/dx)^T H (dF/dx), a 12x12 block, where H is the energy's
// Hessian over vec(F) (Dynamic Deformables, "Hessian Computation"). Clamping H
// per element keeps every block, and so their sum, PSD. A fiber adds its own
// block per tet, so the clamped sum is PSD with no projection of the total.
auto ElasticSystem::ElasticStiffness(const Vec<Real> &x, bool clamped) const
    -> std::vector<Eigen::Triplet<Real>> {
  std::vector<Eigen::Triplet<Real>> triplets;
  triplets.reserve(144 * mesh.t.rows() * (fiber ? 2 : 1));
  ForEachDensity(
      [&](const auto &energy) {
        for (int tt = 0; tt < mesh.t.rows(); ++tt) {
          const Mat3<Real> f = DeformationGradient(x, tt);
          const Mat9<Real> h = clamped
                                   ? energy.ClampedHessian(f, RotationVariantSvd(f))
                                   : energy.Hessian(f);
          const Mat9x12<Real> &dfdx = mesh.PartialFPartialxs()[tt];
          AddTetBlock(triplets, mesh.t, tt,
                      mesh.RestVolumes()[tt] * (dfdx.transpose() * h * dfdx));
        }
      });
  return triplets;
}

auto ElasticSystem::Stiffness(const Vec<Real> &x, bool clamped) const
    -> SparseMat<Real> {
  std::vector<Eigen::Triplet<Real>> triplets = ElasticStiffness(x, clamped);
  for (const SoftContact &c : contacts) {
    const Mat3<Real> block = ContactEnergy(c.vertex)
                                 .Hessian(ContactDofs(x, c))
                                 .topLeftCorner<3, 3>();
    AddBlock(triplets, c.vertex, clamped ? ClampedToPsd(block) : block);
  }
  SparseMat<Real> k(Dofs(), Dofs());
  k.setFromTriplets(triplets.begin(), triplets.end());
  return k;
}

// mass M + stiffness K(x), with K the clamped elastic stiffness.
auto ElasticSystem::RayleighMatrix(const Rayleigh &rayleigh,
                                   const Vec<Real> &x) const
    -> SparseMat<Real> {
  const std::vector<Eigen::Triplet<Real>> triplets = ElasticStiffness(x, true);
  SparseMat<Real> k(Dofs(), Dofs());
  k.setFromTriplets(triplets.begin(), triplets.end());
  return rayleigh.mass * mesh.m + rayleigh.stiffness * k;
}

// Each contact's normal damping, the drag, and Rayleigh's matrix at x.
auto ElasticSystem::MatrixDamping(const Vec<Real> &x) const
    -> SparseMat<Real> {
  std::vector<Eigen::Triplet<Real>> triplets;
  for (const SoftContact &c : contacts) {
    const Vec3<Real> n = UnitNormal(c);
    AddBlock(triplets, c.vertex,
             VertexMass(c.vertex) * contact.damping * n * n.transpose());
  }
  SparseMat<Real> d(Dofs(), Dofs());
  d.setFromTriplets(triplets.begin(), triplets.end());
  if (drag != 0) {
    d += drag * mesh.m;
  }
  if (const auto *rayleigh = std::get_if<Rayleigh>(&damping)) {
    d += RayleighMatrix(*rayleigh, x);
  }
  return d;
}

// -dForces/dv: the matrix damping, plus a DampingEnergy's tet blocks
// V (dF/dx)^T H (dF/dx), with H its Hessian over vec(Fdot).
auto ElasticSystem::DampingMatrix(const Vec<Real> &x) const
    -> SparseMat<Real> {
  SparseMat<Real> d = MatrixDamping(x);
  if (const auto *green = std::get_if<GreenDamping>(&damping)) {
    std::vector<Eigen::Triplet<Real>> triplets;
    triplets.reserve(144 * mesh.t.rows());
    for (int tt = 0; tt < mesh.t.rows(); ++tt) {
      const Mat9<Real> h =
          green->Hessian(DeformationGradient(x, tt), Mat3<Real>::Zero());
      const Mat9x12<Real> &dfdx = mesh.PartialFPartialxs()[tt];
      AddTetBlock(triplets, mesh.t, tt,
                  mesh.RestVolumes()[tt] * (dfdx.transpose() * h * dfdx));
    }
    SparseMat<Real> energy(Dofs(), Dofs());
    energy.setFromTriplets(triplets.begin(), triplets.end());
    d += energy;
  }
  return d;
}

} // namespace sim
