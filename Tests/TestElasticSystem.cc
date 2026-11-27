#include <Solid/ElasticSystem.h>
#include <gtest/gtest.h>

#include <numbers>
#include <random>

using namespace sim;

namespace {

const Lame kLame = Lame::FromYoungs(5.0, 0.45);

auto Materials() -> std::vector<Material> {
  return {StableNeoHookean{kLame}, StVenantKirchhoff{kLame}};
}

// One tet with a unit right-angle corner at the origin.
auto OneTet() -> TetMesh {
  Mat<Real> v(4, 3);
  v << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Mat<int> f(4, 3);
  f << 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3;
  Mat<int> t(1, 4);
  t << 0, 1, 2, 3;
  return {v, f, t};
}

auto Bunny() -> TetMesh {
  return {Meshes / "bunny.obj",
          [](const Mat<Real> &v, const Mat<int> &) { return v; }};
}

// Rest positions moved by a seeded uniform perturbation of up to `amplitude`
// per coordinate.
auto Perturbed(const Vec<Real> &rest, Real amplitude) -> Vec<Real> {
  std::mt19937 rng(7);
  std::uniform_real_distribution<Real> offset(-amplitude, amplitude);
  Vec<Real> x = rest;
  for (Real &coordinate : x) {
    coordinate += offset(rng);
  }
  return x;
}

auto MeanEdgeLength(const TetMesh &mesh) -> Real {
  Real total = 0;
  for (int tt = 0; tt < mesh.t.rows(); ++tt) {
    for (int corner = 1; corner < 4; ++corner) {
      total += (mesh.rv.row(mesh.t(tt, corner)) - mesh.rv.row(mesh.t(tt, 0)))
                   .norm();
    }
  }
  return total / (3 * mesh.t.rows());
}

// The total elastic energy sum_t V_t Psi(F_t), written without ElasticSystem.
auto ElasticEnergy(const TetMesh &mesh, const Material &material,
                   const Vec<Real> &x) -> Real {
  Real energy = 0;
  for (int tt = 0; tt < mesh.t.rows(); ++tt) {
    Mat3<Real> ds;
    for (int corner = 1; corner < 4; ++corner) {
      ds.col(corner - 1) = x.segment<3>(3 * mesh.t(tt, corner)) -
                           x.segment<3>(3 * mesh.t(tt, 0));
    }
    const Mat3<Real> f = ds * mesh.DmInverses()[tt];
    energy += mesh.RestVolumes()[tt] *
              std::visit([&](const auto &e) { return e.Psi(f); }, material);
  }
  return energy;
}

// -dForces/dx at x by central differences, with v = 0.
auto FdStiffness(const ElasticSystem &system, const Vec<Real> &x) -> Mat<Real> {
  const Vec<Real> v = Vec<Real>::Zero(system.Dofs());
  const Real h = 1e-6;
  Mat<Real> fd(system.Dofs(), system.Dofs());
  for (int j = 0; j < system.Dofs(); ++j) {
    Vec<Real> plus = x;
    Vec<Real> minus = x;
    plus(j) += h;
    minus(j) -= h;
    fd.col(j) = -(system.Forces(plus, v) - system.Forces(minus, v)) / (2 * h);
  }
  return fd;
}

const ContactModel kContact{.stiffness = 1e4, .epsilon = 0.02, .damping = 50};

// An equilateral triangle of circumradius 0.05 near each of the first
// `count` vertices of x, with a seeded random normal. The vertex is in turn
// over the face, behind it, and beside an edge, always within kContact's
// epsilon of the triangle.
auto ContactsNear(const Vec<Real> &x, int count) -> std::vector<SoftContact> {
  std::mt19937 rng(11);
  std::normal_distribution<Real> gaussian;
  constexpr Real r = 0.05;
  std::vector<SoftContact> contacts;
  for (int vertex = 0; vertex < count; ++vertex) {
    const Vec3<Real> n =
        Vec3<Real>(gaussian(rng), gaussian(rng), gaussian(rng)).normalized();
    const Vec3<Real> u = n.unitOrthogonal();
    const Vec3<Real> w = n.cross(u);
    const Vec3<Real> p = x.segment<3>(3 * vertex);
    Vec3<Real> center;
    switch (vertex % 3) {
    case 0: center = p - 0.01 * n; break;
    case 1: center = p + 0.01 * n; break;
    default: center = p + (r / 2 + 0.005) * u - 0.008 * n; break;
    }
    SoftContact c{.vertex = vertex, .triangle = {}};
    for (int k = 0; k < 3; ++k) {
      const Real angle = 2 * std::numbers::pi * k / 3;
      c.triangle[k] = center + r * (std::cos(angle) * u + std::sin(angle) * w);
    }
    contacts.push_back(c);
  }
  return contacts;
}

auto SmallestEigenvalue(const SparseMat<Real> &k) -> Real {
  const Mat<Real> dense(k);
  return Eigen::SelfAdjointEigenSolver<Mat<Real>>(dense, Eigen::EigenvaluesOnly)
      .eigenvalues()
      .minCoeff();
}

auto LargestMagnitudeEigenvalue(const SparseMat<Real> &k) -> Real {
  const Mat<Real> dense(k);
  return Eigen::SelfAdjointEigenSolver<Mat<Real>>(dense, Eigen::EigenvaluesOnly)
      .eigenvalues()
      .cwiseAbs()
      .maxCoeff();
}

} // namespace

TEST(TestElasticSystem, ForcesAreMinusEnergyGradient) {
  for (const Material &material : Materials()) {
    const TetMesh mesh = OneTet();
    const ElasticSystem system(mesh, material);
    const Vec<Real> rest = mesh.Positions();
    const Vec<Real> v = Vec<Real>::Zero(system.Dofs());
    EXPECT_LE(system.Forces(rest, v).norm(), 1e-14) << material.index();

    Vec<Real> x = rest;
    x(3) += 0.3;  // vertex 1 stretched along x
    x(7) += 0.2;  // vertex 2 sheared in y
    x(11) -= 0.1; // vertex 3 compressed along z
    const Vec<Real> forces = system.Forces(x, v);

    const Real h = 1e-6;
    Vec<Real> fd(12);
    for (int i = 0; i < 12; ++i) {
      Vec<Real> plus = x;
      Vec<Real> minus = x;
      plus(i) += h;
      minus(i) -= h;
      fd(i) = -(ElasticEnergy(mesh, material, plus) -
                ElasticEnergy(mesh, material, minus)) /
              (2 * h);
    }
    EXPECT_GT(forces.norm(), 0.1) << material.index();
    EXPECT_LE((forces - fd).norm(), 1e-6 * forces.norm()) << material.index();
  }
}

// Energy is the potential Forces descends: -dEnergy/dx = Forces(x, 0), with
// contacts, on the perturbed tet. ElasticEnergy above is the independent
// elastic-only check.
TEST(TestElasticSystem, ForcesAreMinusEnergyGradientWithContacts) {
  for (const Material &material : Materials()) {
    const TetMesh mesh = OneTet();
    ElasticSystem system(mesh, material, kContact);
    const Vec<Real> x = Perturbed(mesh.Positions(), 0.05);
    EXPECT_NEAR(system.Energy(x), ElasticEnergy(mesh, material, x),
                1e-14 * std::abs(system.Energy(x)));
    system.SetContacts(ContactsNear(x, 4));
    const Vec<Real> forces = system.Forces(x, Vec<Real>::Zero(12));
    const Real h = 1e-6;
    Vec<Real> fd(12);
    for (int i = 0; i < 12; ++i) {
      Vec<Real> plus = x;
      Vec<Real> minus = x;
      plus(i) += h;
      minus(i) -= h;
      fd(i) = -(system.Energy(plus) - system.Energy(minus)) / (2 * h);
    }
    EXPECT_GT(forces.norm(), 0.1) << material.index();
    EXPECT_LE((forces - fd).norm(), 1e-6 * forces.norm()) << material.index();
  }
}

// The bunny, deformed by a random perturbation, for every Material.
class BunnyStiffness : public testing::TestWithParam<int> {
protected:
  static void SetUpTestSuite() {
    mesh = std::make_unique<TetMesh>(Bunny());
    x = Perturbed(mesh->Positions(), 0.3 * MeanEdgeLength(*mesh));
  }
  static void TearDownTestSuite() { mesh.reset(); }

  auto MaterialUnderTest() const -> Material { return Materials()[GetParam()]; }

  static std::unique_ptr<TetMesh> mesh;
  static Vec<Real> x;
};
std::unique_ptr<TetMesh> BunnyStiffness::mesh;
Vec<Real> BunnyStiffness::x;

TEST_P(BunnyStiffness, IsMinusForceJacobian) {
  const Material material = MaterialUnderTest();
  const ElasticSystem system(*mesh, material);
  const Mat<Real> k(system.Stiffness(x, false));
  EXPECT_LE((k - FdStiffness(system, x)).norm(), 1e-6 * k.norm());
}

// The contact terms alone, which the elastic terms would otherwise swamp, and
// then the whole matrix.
TEST_P(BunnyStiffness, WithContactsIsMinusForceJacobian) {
  const Material material = MaterialUnderTest();
  const ElasticSystem bare(*mesh, material);
  ElasticSystem system(*mesh, material, kContact, 0.5);
  system.SetContacts(ContactsNear(x, 60));
  const Mat<Real> k(system.Stiffness(x, false));
  const Mat<Real> fd = FdStiffness(system, x);
  const Mat<Real> contactK = k - Mat<Real>(bare.Stiffness(x, false));
  const Mat<Real> contactFd = fd - FdStiffness(bare, x);
  EXPECT_GT(contactK.norm(), 1);
  EXPECT_LE((contactK - contactFd).norm(), 1e-6 * contactK.norm());
  EXPECT_LE((k - fd).norm(), 1e-6 * k.norm());
}

TEST_P(BunnyStiffness, ClampedWithContactsIsPsd) {
  const Material material = MaterialUnderTest();
  ElasticSystem system(*mesh, material, kContact);
  system.SetContacts(ContactsNear(x, 60));
  const SparseMat<Real> clamped = system.Stiffness(x, true);
  EXPECT_GE(SmallestEigenvalue(clamped),
            -1e-10 * LargestMagnitudeEigenvalue(clamped));
}

TEST_P(BunnyStiffness, IsSymmetric) {
  const Material material = MaterialUnderTest();
  const ElasticSystem system(*mesh, material);
  for (const bool clamped : {false, true}) {
    const Mat<Real> k(system.Stiffness(x, clamped));
    EXPECT_LE((k - k.transpose()).norm(), 1e-12 * k.norm()) << clamped;
  }
}

TEST_P(BunnyStiffness, ClampedIsPsdAndUnclampedIsNot) {
  const Material material = MaterialUnderTest();
  const ElasticSystem system(*mesh, material);
  const SparseMat<Real> clamped = system.Stiffness(x, true);
  const SparseMat<Real> unclamped = system.Stiffness(x, false);
  const Real tolerance = 1e-10 * LargestMagnitudeEigenvalue(clamped);
  EXPECT_GE(SmallestEigenvalue(clamped), -tolerance);
  // The control: without clamping, this x makes the stiffness indefinite,
  // far past the round-off the clamped matrix is allowed.
  EXPECT_LT(SmallestEigenvalue(unclamped), -1e3 * tolerance);
}

INSTANTIATE_TEST_SUITE_P(EveryMaterial, BunnyStiffness, testing::Values(0, 1),
                         [](const testing::TestParamInfo<int> &info) {
                           return info.param == 0 ? "StableNeoHookean"
                                                  : "StVenantKirchhoff";
                         });

TEST(TestElasticSystem, DampingMatrixIsZeroUntilDampingExists) {
  const TetMesh mesh = OneTet();
  const Material material = Materials()[0];
  const ElasticSystem system(mesh, material);
  const SparseMat<Real> d = system.DampingMatrix(mesh.Positions());
  EXPECT_EQ(d.rows(), system.Dofs());
  EXPECT_EQ(d.cols(), system.Dofs());
  EXPECT_EQ(d.nonZeros(), 0);
}

TEST(TestElasticSystem, DampingMatrixIsMinusVelocityJacobian) {
  const TetMesh mesh = OneTet();
  const Material material = Materials()[0];
  ElasticSystem system(mesh, material, kContact, 0.5);
  const Vec<Real> x = Perturbed(mesh.Positions(), 0.05);
  system.SetContacts(ContactsNear(x, 4));
  const Vec<Real> v = Vec<Real>::LinSpaced(12, -1, 1);
  const Mat<Real> d(system.DampingMatrix(x));

  const Real h = 1e-6;
  Mat<Real> fd(12, 12);
  for (int j = 0; j < 12; ++j) {
    Vec<Real> plus = v;
    Vec<Real> minus = v;
    plus(j) += h;
    minus(j) -= h;
    fd.col(j) = -(system.Forces(x, plus) - system.Forces(x, minus)) / (2 * h);
  }
  EXPECT_GT(d.norm(), 0);
  EXPECT_LE((d - fd).norm(), 1e-6 * d.norm());
}

TEST(TestConstraints, FilterZeroesPinnedDofsAndIsAProjection) {
  TetMesh mesh = OneTet();
  mesh.PinVertex(1);
  mesh.PinVertex(3);
  const Material material = Materials()[0];
  const ElasticSystem system(mesh, material);
  EXPECT_EQ(system.Constraints().pinned, (std::vector<int>{1, 3}));

  const SparseMat<Real> s = system.Constraints().Filter();
  const Vec<Real> y = Vec<Real>::LinSpaced(12, 1, 12);
  const Vec<Real> filtered = s * y;
  for (int dof = 0; dof < 12; ++dof) {
    const bool pinned = dof / 3 == 1 || dof / 3 == 3;
    EXPECT_EQ(filtered(dof), pinned ? 0 : y(dof)) << dof;
  }
  EXPECT_EQ((Mat<Real>(s * s) - Mat<Real>(s)).norm(), 0);
}
