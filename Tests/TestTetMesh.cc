#include <TetMesh.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace sim;

Mat3<Real> I = Mat3<Real>::Identity();

TEST(TestTetMesh, TestPartialFPartialx0) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp.row(0) << -1, -1, -1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(0, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx1) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp.row(1) << -1, -1, -1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(1, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx2) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp.row(2) << -1, -1, -1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(2, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx3) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(0, 0) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(3, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx4) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(1, 0) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(4, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx5) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(2, 0) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(5, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx6) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(0, 1) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(6, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx7) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(1, 1) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(7, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx8) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(2, 1) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(8, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx9) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(0, 2) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(9, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx10) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(1, 2) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(10, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx11) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(2, 2) = 1;
  auto tetMesh = std::make_unique<TetMesh>();
  Mat3<Real> partial = tetMesh->EvalPartialFPartialx(11, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx) {
  Mat9x12<Real> comp = Mat9x12<Real>::Zero();
  comp << -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0,
      0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  auto tetMesh = std::make_unique<TetMesh>();
  Mat9x12<Real> ret = tetMesh->PartialFPartialx(I);
  EXPECT_EQ(ret, comp);
}
namespace {

// One tet, stretched and sheared, so every Material alternative sees a
// non-trivial F.
auto OneTet() -> TetMesh {
  Mat<Real> v(4, 3);
  v << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Mat<int> f(4, 3);
  f << 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3;
  Mat<int> t(1, 4);
  t << 0, 1, 2, 3;
  return {v, f, t};
}

auto Deformed(const Vec<Real> &rest) -> Vec<Real> {
  Vec<Real> x = rest;
  x(3) += 0.3;  // vertex 1 stretched along x
  x(7) += 0.2;  // vertex 2 sheared in y
  x(11) -= 0.1; // vertex 3 compressed along z
  return x;
}

// The tet's elastic energy: rest volume 1/6 times Psi(F).
auto ElasticEnergy(TetMesh &mesh, const Material &material,
                   const Vec<Real> &x) -> Real {
  mesh.SetPositions(x);
  mesh.ComputeDeformationGradients();
  return std::visit([&](const auto &e) { return e.Psi(mesh.fs[0]) / 6; },
                    material);
}

} // namespace

TEST(TestTetMesh, MaterialForcesAreMinusEnergyGradient) {
  const Lame lame = Lame::FromYoungs(5.0, 0.45);
  for (const Material &material :
       {Material(StableNeoHookean{lame}), Material(StVenantKirchhoff{lame})}) {
    TetMesh mesh = OneTet();
    const Vec<Real> rest = mesh.Positions();

    mesh.ComputeDeformationGradients();
    EXPECT_LE(mesh.ComputeMaterialForces(material).norm(), 1e-14)
        << "index " << material.index();

    const Vec<Real> x = Deformed(rest);
    mesh.SetPositions(x);
    mesh.ComputeDeformationGradients();
    const Vec<Real> forces = mesh.ComputeMaterialForces(material);

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
    EXPECT_GT(forces.norm(), 0.1) << "index " << material.index();
    EXPECT_LE((forces - fd).norm(), 1e-6 * forces.norm())
        << "index " << material.index();
  }
}
