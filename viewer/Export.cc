#include <Export.h>

#include <igl/writeOBJ.h>

#include <format>
#include <fstream>

namespace viewer {

auto ExportFrame(const sim::World &world, const fs::path &directory, int frame)
    -> std::expected<int, std::string> {
  std::error_code ec;
  fs::create_directories(directory, ec);
  if (ec) {
    return std::unexpected(
        std::format("cannot create {}: {}", directory.string(), ec.message()));
  }

  std::vector<const StrandMesh *> strands;
  int written = 0;
  for (int body = 0; body < std::ssize(world.Bodies()); ++body) {
    const auto &state = world.Bodies()[body];
    if (const auto *strand = std::get_if<const StrandMesh *>(&state)) {
      strands.push_back(*strand);
    } else if (const auto *collider =
                   std::get_if<const CollisionMesh *>(&state)) {
      const auto path =
          directory / std::format("collider{}_{:04}.obj", body, frame);
      if (!igl::writeOBJ(path.string(), (*collider)->v, (*collider)->f)) {
        return std::unexpected(std::format("cannot write {}", path.string()));
      }
      ++written;
    }
  }

  if (strands.empty()) {
    return written;
  }

  const auto path = directory / std::format("frame_{:04}.obj", frame);
  std::ofstream file(path);
  for (const auto *strand : strands) {
    const auto &vertices = strand->der->vertices;
    for (int jj = 0; jj < vertices.rows(); ++jj) {
      file << "v " << vertices.row(jj) << '\n';
    }
  }
  int offset = 0;
  for (const auto *strand : strands) {
    const auto rows = strand->der->vertices.rows();
    file << "l ";
    for (int jj = 0; jj < rows; ++jj) {
      file << offset + jj + 1 << ' ';
    }
    file << '\n';
    offset += static_cast<int>(rows);
  }
  if (!file) {
    return std::unexpected(std::format("cannot write {}", path.string()));
  }
  return written + 1;
}

} // namespace viewer
