#include "Golden/Golden.h"

#include <gtest/gtest.h>
#include <igl/readOBJ.h>

#include <fstream>
#include <iostream>
#include <numbers>

using namespace sim;

namespace {

using golden::MakeScene;
using golden::TakeSnapshot;

TEST(World, CatalogScenesConstruct) {
  for (const auto &entry : sim::Catalog()) {
    const auto desc = entry.make();
    EXPECT_EQ(desc.name, entry.name);
    const sim::World world(desc);
    EXPECT_EQ(std::ssize(world.Bodies()), std::ssize(desc.bodies));
  }
}

TEST(World, FrameDebugNeverSteps) {
  sim::World world(MakeScene("frame-debug"));
  const auto before = TakeSnapshot(world);
  EXPECT_TRUE(world.Finished());
  world.Step();
  EXPECT_EQ(world.Steps(), 0);
  EXPECT_TRUE(TakeSnapshot(world)[0] == before[0]);
}

// golden::kRuns stops strand-drop before the comb moves, so check the sweep
// alone: before step s the comb sits at its rest pose plus
// (0, sweep((s - 20001) / 50), 0), with sweep running from 0 down to -7. The
// rest pose is the comb file moved by (-1, -1, 10).
TEST(World, CombSweepFollowsHoldSteps) {
  auto desc = MakeScene("strand-drop");
  const auto collider = std::ranges::find_if(desc.bodies, [](const auto &b) {
    return std::holds_alternative<sim::Collider>(b);
  });
  ASSERT_NE(collider, desc.bodies.end());
  const sim::Body comb = *collider;
  const auto sweep = std::ranges::find_if(desc.script, [](const auto &a) {
    return std::holds_alternative<sim::MoveCollider>(a.effect);
  });
  ASSERT_NE(sweep, desc.script.end());
  sim::Action action = *sweep;
  std::get<sim::MoveCollider>(action.effect).collider = {0};
  desc.bodies = {comb};
  desc.script = {action};
  sim::World world(desc);

  Mat<Real> file;
  Mat<int> faces;
  ASSERT_TRUE(igl::readOBJ((Meshes / "comb_more_teeth.obj").string(), file,
                           faces));
  const auto *mesh = std::get<const CollisionMesh *>(world.Bodies()[0]);
  const Mat<Real> filePose =
      file.rowwise() + Vec3<Real>(-1, -1, 10).transpose();
  ASSERT_LT((mesh->v - filePose).cwiseAbs().maxCoeff(), 1e-12);

  CollisionMesh expected;
  expected.rv = mesh->v;
  expected.v = expected.rv;
  const Vec<Real> drop = Vec<Real>::LinSpaced(200, 0, -7);
  for (int step = 1; step <= 30'000; ++step) {
    world.Step();
    if (step > 20'000) {
      expected.Translate(Vec3<Real>(0, drop((step - 20'001) / 50), 0));
    }
    ASSERT_TRUE(mesh->v == expected.v) << "step " << step;
  }
  EXPECT_TRUE(world.Finished());
}

const sim::Placement kTilted{
    .centroid = Vec3<Real>(1.5, -2, 7),
    .rotation = Eigen::Quaternion<Real>(
        Eigen::AngleAxis<Real>(0.7, Vec3<Real>(1, 2, 3).normalized())),
    .scale = 1.7,
};

auto SignedTetVolume(const Mat<Real> &v, const Vec4<int> &tet) -> Real {
  const Vec3<Real> a = v.row(tet(0));
  return (Vec3<Real>(v.row(tet(1))) - a)
             .dot((Vec3<Real>(v.row(tet(2))) - a)
                      .cross(Vec3<Real>(v.row(tet(3))) - a)) /
         6;
}

auto Bunny(const sim::Placement &place) -> sim::SoftBody {
  return {
      .mesh = {Meshes / "bunny.obj"},
      .place = place,
      .material = sim::StableNeoHookean{.lame = sim::Lame::FromYoungs(5, 0.45)},
      .pinRegion = Eigen::AlignedBox<Real, 3>(Vec3<Real>(-1, 0.3, -1),
                                              Vec3<Real>(1, 1, 1)),
  };
}

auto Hair(const sim::Placement &place) -> sim::Strand {
  Mat<Real> points(6, 3);
  for (int ii = 0; ii < 6; ++ii) {
    points.row(ii) = Vec3<Real>(0.3 * ii, 0.1 * ii * ii, -0.2 * ii);
  }
  return {.points = points, .place = place};
}

auto Placed(sim::Body body) -> sim::World {
  return sim::World(sim::SceneDesc{.name = "placed", .dt = 1e-3,
                                   .bodies = {std::move(body)}});
}

template <class Mesh> auto Only(const sim::World &world) -> const Mesh & {
  return *std::get<const Mesh *>(world.Bodies()[0]);
}

TEST(World, PlacedTetMeshRestsOnItsCentroid) {
  const auto world = Placed(Bunny(kTilted));
  const auto &mesh = Only<TetMesh>(world);
  Vec3<Real> moment = Vec3<Real>::Zero();
  Real volume = 0;
  for (int ii = 0; ii < mesh.t.rows(); ++ii) {
    const Vec4<int> tet = mesh.t.row(ii);
    const Real v = SignedTetVolume(mesh.rv, tet);
    for (int corner = 0; corner < 4; ++corner) {
      moment += v / 4 * Vec3<Real>(mesh.rv.row(tet(corner)));
    }
    volume += v;
  }
  EXPECT_LT((moment / volume - kTilted.centroid).norm(), 1e-12);
  EXPECT_TRUE(mesh.v == mesh.rv);
}

TEST(World, PlacedStrandRestsOnItsCentroid) {
  const auto world = Placed(Hair(kTilted));
  const Mat<Real> &v = Only<StrandMesh>(world).der->vertices;
  EXPECT_LT((Vec3<Real>(v.colwise().mean()) - kTilted.centroid).norm(),
            1e-12);
}

TEST(World, PlacedBoxRestsOnItsCentroid) {
  const auto world = Placed(sim::Collider{
      .shape = sim::Box{.halfExtents = Vec3<Real>(2, 0.5, 1)},
      .place = kTilted});
  const auto &mesh = Only<CollisionMesh>(world);
  ASSERT_EQ(mesh.v.rows(), 8);
  ASSERT_EQ(mesh.f.rows(), 12);
  EXPECT_LT((Vec3<Real>(mesh.v.colwise().mean()) - kTilted.centroid).norm(),
            1e-12);
}

TEST(World, RotationAndScaleKeepTetShape) {
  const auto rest = Placed(Bunny({}));
  const auto tilted = Placed(Bunny(kTilted));
  const auto &a = Only<TetMesh>(rest);
  const auto &b = Only<TetMesh>(tilted);
  ASSERT_TRUE(a.t == b.t);
  const Real s = kTilted.scale;
  for (int ii = 0; ii < a.t.rows(); ++ii) {
    const Vec4<int> tet = a.t.row(ii);
    EXPECT_NEAR(SignedTetVolume(b.rv, tet),
                s * s * s * SignedTetVolume(a.rv, tet), 1e-12);
    for (int jj = 0; jj < 4; ++jj) {
      for (int kk = jj + 1; kk < 4; ++kk) {
        EXPECT_NEAR((b.rv.row(tet(jj)) - b.rv.row(tet(kk))).norm(),
                    s * (a.rv.row(tet(jj)) - a.rv.row(tet(kk))).norm(), 1e-12);
      }
    }
  }
}

TEST(World, RotationAndScaleKeepStrandEdges) {
  const auto strand = Hair(kTilted);
  const auto world = Placed(strand);
  const Mat<Real> &v = Only<StrandMesh>(world).der->vertices;
  for (int ii = 0; ii + 1 < v.rows(); ++ii) {
    EXPECT_NEAR((v.row(ii + 1) - v.row(ii)).norm(),
                kTilted.scale *
                    (strand.points.row(ii + 1) - strand.points.row(ii)).norm(),
                1e-12);
  }
}

TEST(World, PinRegionMovesWithItsBody) {
  const auto rest = Placed(Bunny({}));
  const auto tilted = Placed(Bunny(kTilted));
  const auto &pinned = Only<TetMesh>(rest).pinned;
  EXPECT_GT(pinned.sum(), 0);
  EXPECT_LT(pinned.sum(), pinned.rows());
  EXPECT_TRUE(Only<TetMesh>(tilted).pinned == pinned);
}

TEST(World, BoxFacesWindOutward) {
  const Vec3<Real> h(2, 0.5, 1);
  const auto world = Placed(sim::Collider{
      .shape = sim::Box{.halfExtents = h}, .place = kTilted});
  const auto &mesh = Only<CollisionMesh>(world);
  const Vec3<Real> center = mesh.v.colwise().mean();
  Real volume = 0;
  for (int ii = 0; ii < mesh.f.rows(); ++ii) {
    const Vec3<Real> a = mesh.v.row(mesh.f(ii, 0));
    const Vec3<Real> b = mesh.v.row(mesh.f(ii, 1));
    const Vec3<Real> c = mesh.v.row(mesh.f(ii, 2));
    EXPECT_GT((b - a).cross(c - a).dot((a + b + c) / 3 - center), 0)
        << "face " << ii;
    volume += a.dot(b.cross(c)) / 6;
  }
  const Real s = kTilted.scale;
  EXPECT_NEAR(volume, 8 * h.prod() * s * s * s, 1e-12);
}

TEST(World, NonPositiveScaleIsRejected) {
  auto strand = Hair({});
  strand.place.scale = 0;
  EXPECT_THROW(Placed(strand), std::invalid_argument);
}

TEST(World, PathActionWithoutLastStepIsRejected) {
  auto desc = MakeScene("strand-bend");
  desc.script.front().lastStep.reset();
  EXPECT_THROW(sim::World{desc}, std::invalid_argument);
}

// A regular tet of edge 0.2 sqrt(2) as an OBJ, so it goes through the same
// tetrahedralization as any SoftBody mesh.
auto RegularTetFile() -> sim::fs::path {
  const auto file = sim::fs::path(testing::TempDir()) / "regular_tet.obj";
  std::ofstream out(file);
  out << "v 0.1 0.1 0.1\nv 0.1 -0.1 -0.1\nv -0.1 0.1 -0.1\nv -0.1 -0.1 0.1\n"
         "f 1 2 3\nf 1 4 2\nf 1 3 4\nf 2 4 3\n";
  return file;
}

auto Tet(const sim::Placement &place) -> sim::SoftBody {
  return {.mesh = {RegularTetFile()},
          .place = place,
          .material = sim::StableNeoHookean{.lame = sim::Lame::FromYoungs(50, 0.3)}};
}

// A Box whose top face is the plane through `top` with unit normal `normal`
// (a rotation about z of the +y axis), centered under `top`.
auto Slab(const Vec3<Real> &top, Real angle, const Vec3<Real> &halfExtents)
    -> sim::Collider {
  const Eigen::Quaternion<Real> rotation(
      Eigen::AngleAxis<Real>(angle, Vec3<Real>::UnitZ()));
  return {.shape = sim::Box{.halfExtents = halfExtents},
          .place = {.centroid = top - halfExtents.y() *
                                          (rotation * Vec3<Real>::UnitY()),
                    .rotation = rotation}};
}

auto Centroid(const Mat<Real> &v) -> Vec3<Real> {
  return v.colwise().mean().transpose();
}

auto MaxSpeed(const Mat<Real> &now, const Mat<Real> &before, Real dt) -> Real {
  return (now - before).rowwise().norm().maxCoeff() / dt;
}

// A tilted tet falls 0.3 onto a floor whose top is y = 0. It never sinks past
// the contact epsilon below the top, and comes to rest. Without drag its
// rocking along the frictionless floor and its elastic vibration never decay.
TEST(SoftContact, TetDroppedOnFloorComesToRest) {
  sim::SoftBody tet =
      Tet({.centroid = Vec3<Real>(0, 0.5, 0),
           .rotation = Eigen::Quaternion<Real>(
               Eigen::AngleAxis<Real>(0.4, Vec3<Real>(1, 0, 1).normalized()))});
  tet.drag = 2;
  constexpr Real dt = 1e-3;
  sim::World world(sim::SceneDesc{
      .name = "tet-drop",
      .dt = dt,
      .bodies = {tet, Slab(Vec3<Real>::Zero(), 0, Vec3<Real>(2, 0.25, 2))}});
  const auto &mesh = Only<TetMesh>(world);
  Real lowest = std::numeric_limits<Real>::infinity();
  Mat<Real> before = mesh.v;
  Real speed = 0;
  for (int step = 1; step <= 8000; ++step) {
    world.Step();
    lowest = std::min(lowest, mesh.v.col(1).minCoeff());
    speed = MaxSpeed(mesh.v, before, dt);
    before = mesh.v;
  }
  std::cout << "[ measured ] lowest y " << lowest << ", final max speed "
            << speed << ", final lowest y " << mesh.v.col(1).minCoeff()
            << "\n";
  EXPECT_TRUE(mesh.v.allFinite());
  EXPECT_GT(lowest, -tet.collisionEpsilon);
  EXPECT_LT(speed, 1e-3);
  EXPECT_GT(mesh.v.col(1).minCoeff(), 0);
}

// A tet set face down on a frictionless 30 degree ramp slides with centroid
// acceleration g sin(theta) along the slope once it is in contact.
TEST(SoftContact, TetSlidesDownFrictionlessRampAtGSinTheta) {
  constexpr Real theta = std::numbers::pi / 6;
  constexpr Real dt = 1e-3;
  const Vec3<Real> normal(std::sin(theta), std::cos(theta), 0);
  const Vec3<Real> downhill(std::cos(theta), -std::sin(theta), 0);
  const Vec3<Real> top(0, 1, 0);
  sim::World world(sim::SceneDesc{
      .name = "tet-ramp",
      .dt = dt,
      .bodies = {Tet({.centroid = top - 1.5 * downhill + 0.078 * normal,
                      .rotation = Eigen::Quaternion<Real>::FromTwoVectors(
                          Vec3<Real>(1, 1, 1), normal)}),
                 Slab(top, -theta, Vec3<Real>(3, 0.25, 1))}});
  const auto &mesh = Only<TetMesh>(world);
  const Real g = world.Desc().gravity.norm();

  // Least squares s(t) = a t^2 / 2 + b t + c over the window. Free fall
  // has the same slope component, so the tet must also stay on the ramp.
  constexpr int settle = 1000;
  constexpr int window = 1500;
  Mat<Real> design(window, 3);
  Vec<Real> along(window);
  Vec<Real> height(window);
  for (int step = 1; step <= settle + window; ++step) {
    world.Step();
    if (step > settle) {
      const Real t = (step - settle) * dt;
      design.row(step - settle - 1) << t * t / 2, t, 1;
      along(step - settle - 1) = Centroid(mesh.v).dot(downhill);
      height(step - settle - 1) = (Centroid(mesh.v) - top).dot(normal);
    }
  }
  std::cout << "[ measured ] centroid height over the ramp in "
            << height.minCoeff() << ".." << height.maxCoeff() << "\n";
  EXPECT_GT(height.minCoeff(), 0);
  EXPECT_LT(height.maxCoeff() - height.minCoeff(), 0.01);
  const Real acceleration =
      design.colPivHouseholderQr().solve(along)(0);
  std::cout << "[ measured ] slope acceleration " << acceleration
            << ", g sin(theta) " << g * std::sin(theta) << ", ratio "
            << acceleration / (g * std::sin(theta)) << "\n";
  EXPECT_TRUE(mesh.v.allFinite());
  EXPECT_NEAR(acceleration / (g * std::sin(theta)), 1, 0.02);
}

} // namespace
