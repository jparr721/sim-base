# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An embeddable C++26 physics library. It covers Discrete Elastic Rods (Bergou et al. 2008) with a mass-spring inextensibility constraint, and hyperelastic tet-mesh volumes with ten energies. Both kinds of body collide with static colliders and step with the same five time integrators. Scenes are described as data and run by `sim::World`. A separate GLFW + OpenGL 4.1 + Dear ImGui viewer renders any scene. The core links no GL or windowing library, because a game embeds it.

## Build and test

googletest is a submodule (`git submodule update --init`). libigl v2.6.4 (with Eigen 3.4 and tetgen), GLFW, glad and ImGui come from CMake `FetchContent`.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build -j4                                   # all tests (strand-drop golden takes ~1 min)
ctest --test-dir build -R TestIntegrators                    # one test binary
./build/Tests/TestSceneGolden --gtest_filter='*strandbend*'  # one test case
./build/viewer/sim_viewer --scene strand-drop [--export <dir>]
just demo                                                    # build, then open the viewer on bunny-drop
```

`Lib/CMakeLists.txt` globs the `sim` and `sim_strand` sources, so re-run the configure step after adding or deleting a file there. Every other module lists its files. Project targets build with `-Wall -Wextra -Wpedantic`, and the build stays at zero warnings. Third-party sources compile with `-w`.

Verify the viewer without synthetic input. Never post clicks or keys to the operator's desktop, because the operator works on that machine at the same time. The viewer at this commit takes only `--scene` and `--export`, so check physics headlessly through `World` (`TestWorld`, `examples/embed.cc`) and leave interactive checks to the operator.

## Architecture

Every module is a CMake static library in `Lib/CMakeLists.txt`. The link graph keeps the layers apart, so a stray include fails the build. Dependencies point down this list:

- **Header-only math.** `LibMath.h` (`Real` and the Eigen aliases) and the concept headers. There is no `sim_math` target. Each module reaches these headers through the shared `Lib/` include directory and links Eigen only.
- **Energy modules.** Each links Eigen only.
  - `sim_energy_mass_spring` and `sim_energy_vertex_face_collision` are `ElementEnergy` modules (`Lib/Energy/ElementEnergy.h`) over stacked vertex DOFs.
  - `sim_energy_<name>` is one module per volume energy in `Lib/Energy/Volume/`. The interface target `sim_volume_energies` links all ten.
  - `sim_damping_green` is `GreenDamping`, a `DampingEnergy` over F and Ḟ.
- **`sim_system`.** The `System` concept (`Lib/Solid/System.h`) and `Constraints`, which hold what an integrator sees of a body.
- **`sim_integrators`** (`Lib/Solid/Integrators.h`). Links `sim_system` and Eigen, and never an energy or a mesh.
- **`sim_strand`** (`Lib/Strand/`, `Lib/Energy/Strand/`). `DerSystem` is one strand as a `System`. It links `sim_system`, the mass-spring, collision and damping modules.
- **`sim_solid`.** `TetMesh` and `ElasticSystem`, the tet assembly. It links `sim_system`, the collision module, `sim_volume_energies`, `sim_damping_green` and libigl, and never `sim`.
- **`sim` (alias `sim::sim`).** `World`, the scene description and collision detection. It links every module above.
- **`sim_scenes`** (`SceneCatalog/`) links `sim`. **`sim_viewer`** (`viewer/`) links `sim_scenes`.

The pieces:

- **Namespaces and headers.** Everything is in `namespace sim`. Macros are `SIM_`-prefixed (`SIM_ASSERT`, `SIM_INLINE`). Public headers must stay C++20-clean and must not leak global names. `Tests/PublicHeadersCxx20.cc` and `Tests/EmbedNoCollisions.cc` enforce both by compiling the headers after a game's own `PI`, `E`, `ASSERT` and `Real`. Use `Real` (double) and the Eigen aliases in `LibMath.h` (`Vec<T>`, `Vec3<T>`, `Mat<T>`, `SparseMat<T>`, ...).
- **Scene description (`Lib/Sim/Scene.h`).** A `SceneDesc` holds value types: bodies (`std::variant<Strand, SoftBody, Collider>`) and a script of `Action`s (`MovePinned`, `MoveCollider`, `SetTwistRates`, `SetDamping`). Each action is active over `[firstStep, lastStep]` and runs *before* the step it names integrates. Steps count from 1. Path actions sample `Eigen::LinSpaced` once per `holdSteps`. `ViewHints` is viewer-only data.
- **`World` (`Lib/Sim/World.{h,cc}`).** The only place a `SceneDesc` turns into meshes, systems and integrators. A malformed description throws `std::invalid_argument`. `Step()` runs active actions, then steps strands in parallel (`igl::parallel_for`, one strand per index, no reductions, so results are deterministic). Each strand finds its own collider contacts first. Soft bodies step after that, one at a time. Each finds its contacts once and holds them through its solve. Renderers read state only through `Bodies()` (a variant of const mesh pointers) and `Contacts()`. `StepReports()` says whether each body's last step converged.
- **Integrators.** `ForwardEuler`, `BackwardEuler`, `Newmark`, `Bdf2` and `Quasistatic` step any `System`. The implicit four share one Newton solve. It runs CG on the clamped stiffness and a line search on the incremental potential. A step that does not converge says so in its `StepReport` and keeps the last accepted, finite iterate.
- **Damping (`Lib/Energy/Damping/Damping.h`).** `Damping` is `NoDamping`, `GreenDamping` or `Rayleigh`. A strand cannot take `GreenDamping`, because it has no deformation gradient. Soft-body contact adds damping along the triangle normal and optional mass-proportional `drag`.
- **Contact.** Vertex-face only, frictionless, against static `Collider` triangles. `VertexFaceSqrtCollision` is continuous off the face and across the triangle's plane.

## Authoring a scene

A body is a shape plus a `Placement` (`centroid`, `rotation`, `scale`). `World` recenters the rest shape on its centroid, then scales, rotates and moves it so the centroid lands on `place.centroid`.

- `Strand` points, `SoftBody::pinRegion`, `SoftBody::fiber` and `MoveCollider` paths are body-local, so moving a body means changing `place.centroid` only.
- A `SoftBody` takes a `MeshFile`. A `Collider` takes a `MeshFile` or a `Box`, whose faces wind outward.
- `Strand` and `SoftBody` must name an `integrator` and a `damping`. Neither type has a default constructor, so a body that leaves one out does not compile. `TestWorld` proves this with `static_assert`s.
- `SoftBody::material` is one isotropic `Material`. The optional `SoftBody::fiber` is a `FiberMaterial` that adds to it in every tet. Its direction is uniform, body-local and unit length, and the placement's rotation turns it with the body. A fiber energy alone collapses a body, so it is never the whole material.

Adding a scene means writing one function that returns a `SceneDesc` and adding one registry line to `SceneCatalog/Catalog.cc`. `sim::Catalog()` returns `SceneEntry{name, make}` for these scenes:

- bunny-drop (explicit, dt 1/1000), bunny-drop-implicit and bunny-drop-implicit-damped (BackwardEuler, dt 1/20).
- One ramp scene per energy, each bunny-drop-implicit-damped with its material swapped: bunny-stvk, bunny-linear-elasticity, bunny-neo-hookean, bunny-arap, bunny-symmetric-dirichlet, bunny-snh-barrier, bunny-fiber-stvk, bunny-fiber-arap and bunny-fiber-fung.
- strand-bend, strand-twist, strand-drop (explicit, dt 1/1000), strand-drop-implicit (BackwardEuler, dt 1/60) and frame-debug.

## Adding a volume energy

Write `Lib/Energy/Volume/<File>.{h,cc}` with a struct that satisfies `sim::Energy` (`Lib/Energy/Volume/Energy.h`): `Psi`, `Pk1`, `Hessian` and an analytic `ClampedHessian`. Then add three lines:

1. `sim_volume_energy(<name> <File>)` in `Lib/CMakeLists.txt`.
2. The type as an alternative of `Material` (`Lib/Energy/Volume/Material.h`), or of `FiberMaterial` (`Lib/Energy/Volume/FiberMaterial.h`) for a fiber energy. `ElasticSystem` visits the variant, so it needs no change.
3. The type in `Energies` in `Tests/TestEnergies.cc`, with a `HandDerived<E>` specialization. Add a `Traits<E>` specialization when the energy is not isotropic or is not defined for every F.

A catalog scene for the energy is one `BunnyEnergy` call in `SceneCatalog/Catalog.cc`, plus its golden.

## Tests that pin behavior

Most test binaries link one module and nothing above it, so a module that starts to need `sim` fails to link.

- **`TestEnergies`** links `sim_volume_energies` only. On 1,000 random F, about half inverted, every volume energy must pass: `Pk1` against finite differences of `Psi`, `Hessian` against finite differences of `Pk1`, a PSD `ClampedHessian` that equals the numerical PSD projection, its rest states and rotation symmetry, and one hand-derived value. It also pins the Neo-Hookean guard, the SNH barrier and `RotationVariantSvd`.
- **`TestElementEnergies`** links the mass-spring and collision modules only. It checks gradient and Hessian against finite differences, a PSD clamped Hessian, and that the contact energy stays continuous off the face and across its plane.
- **`TestDamping`** links `sim_damping_green` only. `GreenDamping`'s stress and Hessian match finite differences in Ḟ, the Hessian is PSD, and a rigid rotation rate gives no stress.
- **`TestElasticSystem`** links `sim_solid` only. On the bunny, forces are minus the energy gradient and stiffness is minus the force Jacobian, with and without contacts and fibers. The clamped stiffness is PSD where the unclamped one is not. The damping matrix is minus the velocity Jacobian, and the constraint filter is a projection.
- **`TestDerSystem`** links `sim_strand` and `sim_integrators`. Bending, twist, spring and contact stiffness match finite differences of the strand forces. Clamped blocks are PSD, contacts push out of a counterclockwise triangle, and the old velocity filter equals Rayleigh mass damping.
- **`TestIntegrators`** links `sim_integrators` and `sim_solid`, on a tet and on the bunny. BackwardEuler at dt 1/60 is stable and dissipative where ForwardEuler blows up. Newmark keeps energy within 1%. Halving dt cuts Bdf2's error by 3 to 5 times. Quasistatic matches BackwardEuler run to rest. An unconverged step reports failure. Each method damps at its own rate.
- **`TestWorld`** covers description validation, placement, action timing, the required fields, soft-body contact on a floor and a frictionless ramp, bunny-drop, bunny-drop-implicit and each energy scene coming to rest past the ramp's foot, and strand-drop-implicit converging on every step.
- **`TestSceneGolden`** replays each stepping catalog scene through `World` and compares checkpoint positions with `Tests/Golden/*.txt` to 1e-12 relative. Any non-finite value is a failure. After a deliberate physics change, regenerate with `./build/Tests/GenerateGolden` and say why in the commit.
- **`TestStrandForces`** checks bending, twist and spring forces against finite differences of independently written energies.
- **`EmbedHeadless`** (`examples/embed.cc`) steps a scene with no window and links no GL.

## Plans and records

`docs/plans/` holds the modernization plan, the DER math audit (what was wrong and the evidence), and the solid-mechanics plan for HOBAK-style energies and integrators, with each phase's result. HOBAK is GPLv3 with a non-commercial clause, so take its structure only and never its code. The operator confirmed this on 2026-11-26. Derive from *Dynamic Deformables* and the cited papers. Copy no HOBAK code, adapted or not, and do not open HOBAK source.
