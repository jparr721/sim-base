#include "Golden/Golden.h"

#include <gtest/gtest.h>
#include <igl/readOBJ.h>

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

TEST(World, PathActionWithoutLastStepIsRejected) {
  auto desc = MakeScene("strand-bend");
  desc.script.front().lastStep.reset();
  EXPECT_THROW(sim::World{desc}, std::invalid_argument);
}

} // namespace
