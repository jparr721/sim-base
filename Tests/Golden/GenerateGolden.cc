#include "Golden.h"

#include <iostream>

auto main() -> int {
  for (const auto &run : golden::kRuns) {
    const auto path = golden::GoldenPath(run.name);
    std::ofstream out(path);
    golden::RunScene(run, [&](int step, const golden::Snapshot &snapshot) {
      out << golden::Serialize(step, snapshot);
    });
    std::cout << "wrote " << path << '\n';
  }
}
