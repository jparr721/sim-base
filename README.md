# sim-base

An embeddable C++ physics library with a standalone viewer.

- Discrete Elastic Rods (Bergou et al. 2008) with a mass-spring inextensibility constraint and vertex-face collisions
- Hyperelastic tet-mesh volumes (Stable Neo-Hookean)

Scenes are plain data (`sim::SceneDesc`) run by `sim::World`. The `sim` library links no graphics or windowing code.

## Build

```sh
git submodule update --init
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build -j4
```

Needs CMake 3.25+ and a C++26 compiler. Public headers also compile as C++20.

## Run the viewer

```sh
./build/viewer/sim_viewer --scene strand-drop
```

| Input | Action |
|---|---|
| `a` | play/pause |
| `space` | single step |
| `g` | toggle grid |
| arrow keys | pull |
| left drag | orbit |
| right drag or scroll | zoom |
| middle or shift-left drag | pan |

`--export <dir>` writes OBJ frames for `Scripts/mitsuba.py`.

## Embed

```cpp
#include <Sim/World.h>
#include <SceneCatalog/Catalog.h>

sim::World world(sim::Catalog()[0].make());
while (!world.Finished()) {
  world.Step();
}
for (const auto &body : world.Bodies()) { /* read positions */ }
```

Link `sim::sim`, and add `sim_scenes` for the built-in scenes. `examples/embed.cc` is a complete headless example, and `ctest -R EmbedHeadless` runs it.
