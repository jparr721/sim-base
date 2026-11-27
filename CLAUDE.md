# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An embeddable C++26 physics library. It covers Discrete Elastic Rods (Bergou et al. 2008) with a mass-spring inextensibility constraint and vertex-face collisions, plus hyperelastic tet-mesh volumes (Stable Neo-Hookean). Scenes are described as data and run by `sim::World`. A separate GLFW + OpenGL 4.1 + Dear ImGui viewer renders any scene. The core links no GL or windowing library, because a game embeds it.

## Build and test

googletest is a submodule (`git submodule update --init`). libigl v2.6.4 (with Eigen 3.4 and tetgen), GLFW, glad and ImGui come from CMake `FetchContent`.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build -j4                                   # all tests (strand-drop golden takes ~1 min)
ctest --test-dir build -R TestStrandForces                   # one test binary
./build/Tests/TestSceneGolden --gtest_filter='*strandbend*'  # one test case
./build/viewer/sim_viewer --scene strand-drop [--export <dir>]
```

`Lib/CMakeLists.txt` uses `file(GLOB)`, so re-run the configure step after adding or deleting a source file. Project targets build with `-Wall -Wextra -Wpedantic`, and the build stays at zero warnings. Third-party sources compile with `-w`.

## Architecture

Targets, by dependency direction: `sim_energy_mass_spring` ← `sim` ← `sim_scenes` ← `sim_viewer`.

- **`sim` (Lib/).** Everything is in `namespace sim`. Macros are `SIM_`-prefixed (`SIM_ASSERT`, `SIM_INLINE`). Public headers must stay C++20-clean and must not leak global names. `Tests/PublicHeadersCxx20.cc` and `Tests/EmbedNoCollisions.cc` enforce both by compiling the headers after a game's own `PI`, `E`, `ASSERT` and `Real`. Use `Real` (double) and the Eigen aliases in `LibMath.h` (`Vec<T>`, `Vec3<T>`, `Mat<T>`, `SparseMat<T>`, ...).
- **Scene description (`Lib/Sim/Scene.h`).** A `SceneDesc` holds value types: bodies (`std::variant<Strand, SoftBody, Collider>`) and a script of `Action`s (`MovePinned`, `MoveCollider`, `SetTwistRates`, `SetDamping`). Each action is active over `[firstStep, lastStep]` and runs *before* the step it names integrates. Steps count from 1. Path actions sample `Eigen::LinSpaced` once per `holdSteps`. `ViewHints` is viewer-only data.
- **`World` (`Lib/Sim/World.{h,cc}`).** The only place a `SceneDesc` turns into meshes and integrators. A malformed description throws `std::invalid_argument`. `Step()` runs active actions, detects strand-vs-collider contacts, sets gravity, then integrates strands in parallel (`igl::parallel_for`, one strand per index, no reductions, so results are deterministic). Renderers read state only through `Bodies()` (a variant of const mesh pointers) and `Contacts()`.
- **Energies (`Lib/Energy/`).** Strand energies live in `DiscreteElasticRod` and `AdaptedFrame`. `MassSpring` is its own module that satisfies `ElementEnergy<E, N>` (`Lib/Energy/ElementEnergy.h`), with `Psi`, `Gradient`, `Hessian` and an analytically `ClampedHessian` over stacked vertex DOFs. Volume energies are under `Lib/Energy/Volume`.
- **Integrators (`Lib/Integrator/`).** `ForwardEulerStrand` and `ForwardEulerVolume`.
- **`sim_scenes` (`SceneCatalog/`).** `sim::Catalog()` returns `SceneEntry{name, make}` for bunny-drop, strand-bend, strand-twist, strand-drop and frame-debug. Adding a scene means writing one function that returns a `SceneDesc` and adding one registry line.
- **`sim_viewer` (`viewer/`).**
  - Window: GLFW, a 4.1 core context, glad.
  - Renderer: unlit lines/points and lit triangles, VBOs refilled from `World::Bodies()` each frame.
  - Camera: `Camera.h`.
  - App: the frame loop.
  - ui/: each ImGui panel is a free function over `ViewerState&`, registered in `ui::Panels()`.
  - Export: writes `frame_NNNN.obj` strand polylines and `collider<body>_NNNN.obj` meshes, which `Scripts/mitsuba.py` renders.

## Tests that pin behavior

- **`TestSceneGolden`** replays each catalog scene through `World` and compares checkpoint positions with `Tests/Golden/*.txt` to 1e-12 relative. Any non-finite value is a failure. After a deliberate physics change, regenerate with `./build/Tests/GenerateGolden` and say why in the commit.
- **`TestStrandForces`** checks bending, twist and spring forces against finite differences of independently written energies.
- **`TestElementEnergies`** checks every `ElementEnergy`: gradient and Hessian against finite differences, and the clamped Hessian is PSD.
- **`TestWorld`** covers description validation and action timing.
- **`EmbedHeadless`** (`examples/embed.cc`) steps a scene with no window and links no GL.

## Plans and records

`docs/plans/` holds the modernization plan, the DER math audit (what was wrong and the evidence), and the solid-mechanics plan for HOBAK-style energies and integrators. HOBAK is GPLv3 with a non-commercial clause, so take its structure only and never its code.
