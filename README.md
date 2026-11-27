# sim-base

An embeddable C++ physics library with a standalone viewer.

- **Strands.** Discrete Elastic Rods (Bergou et al. 2008) with a mass-spring inextensibility constraint.
- **Volumes.** Tet meshes with seven isotropic energies: Stable Neo-Hookean, Stable Neo-Hookean with an inversion barrier, St. Venant-Kirchhoff, linear elasticity, Neo-Hookean (Bonet and Wood), ARAP and symmetric Dirichlet. Three fiber energies add stiffness along one direction: anisotropic StVK, anisotropic ARAP and Fung. Each energy has an analytic PSD-clamped Hessian.
- **Integrators.** Forward Euler, Backward Euler, Newmark, BDF2 and quasistatic. Strands and volumes use the same ones. The implicit methods run Newton with conjugate gradients on the clamped stiffness and report when a step does not converge.
- **Damping.** Green-strain damping, Rayleigh damping, or none.
- **Contact.** Frictionless vertex-face contact against static colliders, from a mesh file or a box.

Scenes are plain data (`sim::SceneDesc`) run by `sim::World`. Every body is a shape plus a `Placement`, and must name its integrator and damping. The `sim` library links no graphics or windowing code.

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
just demo                                      # build, then open bunny-drop
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

`--export <dir>` writes OBJ frames for `Scripts/mitsuba.py`. `just demo --export <dir>` passes the flag through.

## Embed

This program drops a bunny on a floor at a 60 Hz tick and stops after 2 s:

```cpp
#include <Settings.h>
#include <Sim/World.h>

#include <cstdio>

auto main() -> int {
  sim::SceneDesc desc{
      .name = "bunny-on-floor",
      .dt = 1.0 / 60,
      .bodies = {
          sim::SoftBody{
              .mesh = {sim::Meshes / "bunny.obj"},
              .place = {.centroid = sim::Vec3<sim::Real>(0, 1, 0)},
              .material = sim::StableNeoHookean{sim::Lame::FromYoungs(5, 0.45)},
              .integrator = sim::BackwardEuler{.maxNewtonIterations = 25},
              .damping = sim::GreenDamping{.mu = 2},
          },
          sim::Collider{
              .shape = sim::Box{sim::Vec3<sim::Real>(2, 0.25, 2)},
              .place = {.centroid = sim::Vec3<sim::Real>(0, -0.25, 0)},
          },
      },
      .lastStep = 120,
  };
  sim::World world(std::move(desc));
  while (!world.Finished()) {
    world.Step();
    if (!world.StepReports()[0].converged) {
      return 1; // Newton did not converge on this step
    }
  }
  const auto *bunny = std::get<const sim::TetMesh *>(world.Bodies()[0]);
  std::printf("steps %d, lowest y %.4f\n", world.Steps(),
              bunny->v.col(1).minCoeff());
  return bunny->v.allFinite() ? 0 : 1;
}
```

`lastStep` bounds the run. A scene without it never finishes. `World::Bodies()` gives each body's mesh in `SceneDesc::bodies` order, here a `TetMesh` with positions in `bunny->v`. `sim::Meshes` is the repository's `Meshes/` directory. A game passes its own path.

Link `sim::sim`, and add `sim_scenes` for the built-in scenes in `sim::Catalog()`. `examples/embed.cc` is a complete headless example, and `ctest -R EmbedHeadless` runs it.
