// Headless embedding: step a catalog scene with no window or GL.
#include <SceneCatalog/Catalog.h>
#include <Sim/World.h>

#include <cmath>
#include <variant>

auto main() -> int {
  sim::World world(sim::Catalog()[0].make());
  for (int step = 0; step < 100 && !world.Finished(); ++step) {
    world.Step();
  }
  for (const auto &body : world.Bodies()) {
    const bool finite = std::visit(
        [](const auto *mesh) {
          using Mesh = std::remove_cvref_t<decltype(*mesh)>;
          if constexpr (std::is_same_v<Mesh, sim::StrandMesh>) {
            return mesh->der->vertices.allFinite();
          } else {
            return mesh->v.allFinite();
          }
        },
        body);
    if (!finite) {
      return 1;
    }
  }
  return world.Steps() == 100 ? 0 : 1;
}
