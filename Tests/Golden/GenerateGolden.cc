#include "Golden.h"

#include <iostream>

auto main() -> int {
  for (const auto &run : golden::kRuns) {
    std::ofstream out(golden::GoldenPath(run.name));
    golden::RunScene(run, [&](int step, const golden::Snapshot &snapshot) {
      out << golden::Serialize(step, snapshot);
    });
    std::cout << "wrote " << golden::GoldenPath(run.name) << '\n';
  }
}
