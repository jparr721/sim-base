#pragma once

#include <expected>
#include <optional>
#include <string>

namespace viewer {

struct Options {
  std::string scene = "bunny-drop"; // catalog name
  std::optional<std::string> exportDirectory; // export every rendered frame
};

auto ParseOptions(int argc, char **argv) -> std::expected<Options, std::string>;

// Opens the window and runs the frame loop until it closes. Returns the
// process exit code.
auto Run(const Options &options) -> int;

} // namespace viewer
