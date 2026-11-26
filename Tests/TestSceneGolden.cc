#include "Golden/LegacyScenes.h"

#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace {

auto ReadLines(const fs::path &path) -> std::vector<std::string> {
  std::ifstream in(path);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

// strtod, unlike operator>>, reads "nan" and "inf", so a blown-up run cannot
// truncate both sides into a vacuous match.
auto Parse(const std::string &line) -> std::vector<double> {
  std::vector<double> values;
  const char *cursor = line.c_str();
  for (char *end = nullptr;; cursor = end) {
    const double x = std::strtod(cursor, &end);
    if (end == cursor) {
      return values;
    }
    values.push_back(x);
  }
}

class SceneGolden : public testing::TestWithParam<golden::Run> {};

TEST_P(SceneGolden, MatchesRecordedTrajectory) {
  const auto &run = GetParam();
  const auto expected = ReadLines(golden::GoldenPath(run.name));
  ASSERT_FALSE(expected.empty()) << "missing golden file for " << run.name;

  std::vector<std::string> actual;
  golden::RunLegacy(run, [&](int step, const golden::Snapshot &snapshot) {
    std::istringstream lines(golden::Serialize(step, snapshot));
    for (std::string line; std::getline(lines, line);) {
      actual.push_back(line);
    }
  });

  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t ii = 0; ii < expected.size(); ++ii) {
    const auto want = Parse(expected[ii]);
    const auto got = Parse(actual[ii]);
    ASSERT_EQ(got.size(), want.size()) << "line " << ii;
    for (std::size_t jj = 0; jj < want.size(); ++jj) {
      ASSERT_TRUE(std::isfinite(got[jj]))
          << run.name << " step " << want[0] << " body " << want[1]
          << " value " << jj << " is not finite";
      ASSERT_NEAR(got[jj], want[jj], 1e-12 * std::max(1.0, std::abs(want[jj])))
          << run.name << " step " << want[0] << " body " << want[1]
          << " value " << jj;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Scenes, SceneGolden, testing::ValuesIn(golden::kRuns),
                         [](const auto &info) {
                           auto name = info.param.name;
                           std::erase(name, '-');
                           return name;
                         });

} // namespace
