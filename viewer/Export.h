#pragma once

#include <Sim/World.h>

#include <expected>
#include <string>

namespace viewer {

// Writes one frame into `directory`, creating it if needed:
//   frame_NNNN.obj              every strand as an OBJ polyline ("v" / "l")
//   collider<body>_NNNN.obj     each collider's triangle mesh
// Returns the number of files written.
auto ExportFrame(const sim::World &world, const sim::fs::path &directory, int frame)
    -> std::expected<int, std::string>;

} // namespace viewer
