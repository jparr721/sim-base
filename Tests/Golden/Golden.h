#pragma once

// Checkpoint snapshots of the catalog scenes. Tests/Golden/*.txt pin them.

#include <SceneCatalog/Catalog.h>
#include <Sim/World.h>

#include <algorithm>
#include <format>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace golden {

using sim::fs::path;
using sim::Mat;
using sim::Real;

struct Run {
  std::string name;
  int steps;
  int checkpointEvery;
};

inline const std::vector<Run> kRuns = {
    {"bunny-drop", 14000, 1000},
    {"bunny-drop-implicit", 280, 20},
    {"strand-bend", 7000, 500},
    {"strand-twist", 3000, 250},
    {"strand-drop", 20500, 2500},
};

using Snapshot = std::vector<Mat<Real>>;
using Recorder = std::function<void(int step, const Snapshot &)>;

inline auto MakeScene(std::string_view name) -> sim::SceneDesc {
  const auto catalog = sim::Catalog();
  const auto entry = std::ranges::find(catalog, name, &sim::SceneEntry::name);
  if (entry == catalog.end()) {
    throw std::invalid_argument(std::string(name) + " is not in the catalog");
  }
  return entry->make();
}

// Every body's vertex positions, in SceneDesc::bodies order.
inline auto TakeSnapshot(const sim::World &world) -> Snapshot {
  Snapshot snapshot;
  for (const auto &body : world.Bodies()) {
    std::visit(
        [&](const auto *mesh) {
          using Mesh = std::remove_cvref_t<decltype(*mesh)>;
          if constexpr (std::is_same_v<Mesh, sim::StrandMesh>) {
            snapshot.push_back(mesh->der->vertices);
          } else {
            snapshot.push_back(mesh->v);
          }
        },
        body);
  }
  return snapshot;
}

inline void RunScene(const Run &run, const Recorder &record) {
  sim::World world(MakeScene(run.name));
  for (int step = 1; step <= run.steps; ++step) {
    world.Step();
    if (step % run.checkpointEvery == 0) {
      record(step, TakeSnapshot(world));
    }
  }
}

inline auto GoldenPath(const std::string &name) -> path {
  return path(__FILE__).parent_path() / (name + ".txt");
}

// One line per checkpoint body: "<step> <body> <x0> <y0> <z0> <x1> ...".
inline auto Serialize(int step, const Snapshot &snapshot) -> std::string {
  std::string out;
  for (int body = 0; body < std::ssize(snapshot); ++body) {
    out += std::format("{} {}", step, body);
    const auto &m = snapshot[body];
    for (int r = 0; r < m.rows(); ++r) {
      for (int c = 0; c < m.cols(); ++c) {
        out += std::format(" {:.17g}", m(r, c));
      }
    }
    out += '\n';
  }
  return out;
}

} // namespace golden
