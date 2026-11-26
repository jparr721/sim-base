#include "Golden/LegacyScenes.h"

#include <SceneCatalog/Catalog.h>
#include <Sim/World.h>

#include <gtest/gtest.h>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <map>

namespace {

auto MakeScene(std::string_view name) -> sim::SceneDesc {
  const auto catalog = sim::Catalog();
  const auto entry = std::ranges::find(catalog, name, &sim::SceneEntry::name);
  if (entry == catalog.end()) {
    throw std::invalid_argument(std::string(name) + " is not in the catalog");
  }
  return entry->make();
}

// Every body's vertex positions, in SceneDesc::bodies order.
auto Snapshot(const sim::World &world) -> golden::Snapshot {
  golden::Snapshot snapshot;
  for (const auto &body : world.Bodies()) {
    std::visit(
        [&](const auto *mesh) {
          using Mesh = std::remove_cvref_t<decltype(*mesh)>;
          if constexpr (std::is_same_v<Mesh, StrandMesh>) {
            snapshot.push_back(mesh->der->vertices);
          } else {
            snapshot.push_back(mesh->v);
          }
        },
        body);
  }
  return snapshot;
}

auto SameBits(double a, double b) -> bool {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b) ||
         (std::isnan(a) && std::isnan(b));
}

class WorldMatchesLegacy : public testing::TestWithParam<golden::Run> {};

TEST_P(WorldMatchesLegacy, EveryCheckpointIsBitIdentical) {
  const auto &run = GetParam();

  std::map<int, golden::Snapshot> legacy;
  golden::RunLegacy(run, [&](int step, const golden::Snapshot &snapshot) {
    legacy[step] = snapshot;
  });
  ASSERT_FALSE(legacy.empty()) << "no legacy replica for " << run.name;

  sim::World world(MakeScene(run.name));
  int checkpoints = 0;
  for (int step = 1; step <= run.steps; ++step) {
    world.Step();
    ASSERT_EQ(world.Steps(), step);
    if (step % run.checkpointEvery != 0) {
      continue;
    }
    ++checkpoints;
    const auto got = Snapshot(world);
    const auto &want = legacy.at(step);
    ASSERT_EQ(got.size(), want.size()) << "step " << step;
    for (std::size_t body = 0; body < want.size(); ++body) {
      ASSERT_EQ(got[body].size(), want[body].size())
          << "step " << step << " body " << body;
      for (Eigen::Index ii = 0; ii < want[body].size(); ++ii) {
        const double a = got[body](ii);
        const double b = want[body](ii);
        ASSERT_TRUE(SameBits(a, b))
            << run.name << " step " << step << " body " << body << " value "
            << ii << ": world " << std::format("{:.17g}", a) << " legacy "
            << std::format("{:.17g}", b) << " relative "
            << std::abs(a - b) / std::max(1.0, std::abs(b));
      }
    }
  }
  EXPECT_EQ(checkpoints, std::ssize(legacy));
}

INSTANTIATE_TEST_SUITE_P(Scenes, WorldMatchesLegacy,
                         testing::ValuesIn(golden::kRuns),
                         [](const auto &info) {
                           auto name = info.param.name;
                           std::erase(name, '-');
                           return name;
                         });

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
  const auto before = Snapshot(world);
  EXPECT_TRUE(world.Finished());
  world.Step();
  EXPECT_EQ(world.Steps(), 0);
  EXPECT_TRUE(Snapshot(world)[0] == before[0]);
}

// golden::kRuns stops strand-drop before the comb moves, so check the sweep
// alone against the legacy combTranslation lookup: before step s the comb sat
// at combTranslation(sceneFrames), with sceneFrames = (s - 20001) / 50.
TEST(World, CombSweepMatchesLegacyFrameCounter) {
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

  Comb legacy;
  const Vec<Real> combTranslation = Vec<Real>::LinSpaced(200, -8, -1).reverse();
  const auto *mesh = std::get<const CollisionMesh *>(world.Bodies()[0]);
  legacy.Translate(Vec3<Real>(-1, -1, 10));
  ASSERT_TRUE(mesh->v == legacy.v);
  for (int step = 1; step <= 30'000; ++step) {
    world.Step();
    if (step > 20'000) {
      legacy.Translate(
          Vec3<Real>(-1, combTranslation((step - 20'001) / 50), 10.0));
    }
    ASSERT_TRUE(mesh->v == legacy.v) << "step " << step;
  }
  EXPECT_TRUE(world.Finished());
}

TEST(World, PathActionWithoutLastStepIsRejected) {
  auto desc = MakeScene("strand-bend");
  desc.script.front().lastStep.reset();
  EXPECT_THROW(sim::World{desc}, std::invalid_argument);
}

} // namespace
