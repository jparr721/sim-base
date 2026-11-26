# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A C++17 physics simulation sandbox: nonlinear hyperelastic volumes (StVK, Stable Neo-Hookean) on tet meshes, and Discrete Elastic Rods with a mass-spring inextensibility constraint and vertex-face collisions against a comb mesh. Rendering is legacy OpenGL + GLUT. Offline renders go through Mitsuba via Python scripts.

## Build and test

Dependencies: GLUT and OpenGL must be installed on the system. libigl (with Eigen and the copyleft tetgen module) is fetched by CMake `FetchContent` at a pinned commit. googletest is a git submodule at `ext/googletest`; run `git submodule update --init` first or the test build fails.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build                          # all tests
ctest --test-dir build -R TestSNH               # one test binary
./build/Tests/TestSNH --gtest_filter='TestSNH.TestHessianConverges'   # one test case
```

Pass `-DBUILD_TESTING=OFF` to skip tests. There is no lint or format config in the repo.

Executables: `viewer` (from `main.cc`) plus one per file in `Scenes/`, registered with the `package_add_scene` macro in the root `CMakeLists.txt`. New tests are registered with `package_add_test` in `Tests/CMakeLists.txt`.

## Architecture

All simulation code lives in `Lib/` and builds into one static library `base`. `Lib/CMakeLists.txt` globs each subdirectory explicitly, so a new subdirectory under `Lib/` must be added there. Include paths are relative to `Lib/` (e.g. `#include <Energy/Volume/SNH.h>`).

The layering runs mesh → energy → integrator → scene → GLUT driver:

- **Math types** (`LibMath.h`, `Settings.h`). `Real` is `double`. Use the Eigen aliases `Vec<T>`, `Vec3<T>`, `Mat<T>`, `Mat3<T>`, etc. instead of raw Eigen types. `Settings.h` defines `INLINE`, `ASSERT`, and the `Meshes` path (resolved from `__FILE__`, so mesh loading works from any working directory).
- **Meshes** (`TetMesh`, `StrandMesh`, `Collision/CollisionMesh`). `StrandMesh` wraps a `DiscreteElasticRod` (`der`) that owns the vertex positions; DOFs are `3 * nVertices`, flattened.
- **Energies** (`Energy/`). Volume materials derive from `HyperelasticMaterial` and implement `Psi`, `Pk1`, and `Hessian` of the deformation gradient `F`. The base class provides `FiniteDifferenceTestPk1/Hessian`, which is what the tests call. Strand energies (`DiscreteElasticRod`, `DerBending`, `AdaptedFrame`) handle bending and twist via material frames and holonomy. `MassSpring` enforces inextensibility. Collision energies are in `Energy/Collision`.
- **Integrators** (`Integrator/`, header-only). `VolumeIntegrator` and `StrandIntegrator` are abstract bases with a `Step()`; forward and backward Euler subclasses implement them. Strand integrators default to `dt = 1/3000`.
- **Scenes** (`Lib/Scenes/`). A scene struct owns meshes and integrators and exposes `Step`/`StepScriptedScene(gravity)`, `Draw()`, and `DumpFrame()`. Concrete scenes are subclasses whose constructors build the setup (e.g. `DiscreteElasticRods : StrandDropScene`).
- **Drivers** (`main.cc`, `Scenes/*.cc`). Each is a self-contained GLUT program with global camera and input state that instantiates one scene and calls it from the idle and display callbacks. The driver files duplicate most of their GLUT boilerplate. Swapping the scene in a driver means editing the `gScene` line.

## Rendering pipeline

Running a driver with `--save` calls `DumpFrame()` every 50 steps, writing strand OBJs to a hardcoded absolute path in `Lib/Scenes/StrandDropScene.cc`. `Scripts/mitsuba*.py` are Typer CLIs that turn those OBJ frames into Mitsuba scenes (`pip install -r Scripts/requirements.txt`). `Prototyping/` holds standalone Python experiments.
