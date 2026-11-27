# Plan: HOBAK-style energies and time integration, then more solid-mechanics energies

## What changes

sim-base's volume simulation moves to the architecture of HOBAK (Kim & Eberle, "Dynamic Deformables", SIGGRAPH 2022 course).

- **Energies.** Each energy is a pure function of the deformation gradient F with four evaluations: energy, PK1 stress, Hessian, and an analytically clamped (PSD) Hessian.
- **Damping.** Damping is its own energy, a function of F and its time derivative Ḟ.
- **Integrators.** Every time integrator shares one assembly of forces and stiffness, and handles constraints by projection.

That replaces `HyperelasticMaterial` (a virtual base with finite-difference self-tests baked in), `ForwardEulerVolume`, and the empty `BackwardEulerVolume` stub. After that base exists, the energy catalog grows one energy per PR.

## License constraint (decides how we "pull in" HOBAK)

HOBAK's `COPYING` is GPLv3, and every source header adds "permission is granted to use this software solely for non-commercial applications." Copying any HOBAK source into sim-base would bind this library, and the voxel game that embeds it, to both terms.

**Rule for every PR below: no HOBAK code, comments or file layout is copied.** We adopt the architecture, meaning which functions exist and how responsibilities split, which isn't copyrightable. Each energy is derived fresh from the published math: the course notes, Smith et al. 2018 "Stable Neo-Hookean Flesh Simulation", Smith et al. 2019 "Analytic Eigensystems for Isotropic Distortion Energies", and Kim et al. 2019 "Anisotropic Elasticity for Inversion-Safety and Element Rehabilitation". Reviewers check each PR for copied code.

**Confirmed 2026-11-26 (operator):** derive from *Dynamic Deformables* and the cited papers. No HOBAK code goes into the repo, adapted or not. Workers don't open HOBAK source at all.

## Run order (2026-11-26)

Write sets decide the waves. One shared file puts two tasks in different waves.

1. **Wave 1, in parallel:**
   - S1 (Scene.h, World, TetMesh construction, Catalog).
   - H1a (new energy files, `Energy.h`, the SVD, `TestEnergies`). H1a is the new-files half of H1.
2. **Wave 2:** H1b wires `Material` into TetMesh, deletes `HyperelasticMaterial`, `SNH.*`, `STVK.*` and `TestSNH`, and pins the bunny golden.
3. **Then, one at a time:** H2, C1, H3.
4. **After H3:**
   - H4 and H5 run in parallel, since their write sets are disjoint.
   - E1 to E6 energies fan out as one swarm, since each is its own module plus one type in the typed suite.

## Modules (operator decision, 2026-11-26)

Every energy and every integrator is its own module: a directory plus a CMake static library. The link graph enforces the decoupling, so a stray include fails the build.

```
sim_math            LibMath, Svd, Lame, concepts (Energy, DampingEnergy, System)
sim_energy_<name>   one per energy: links sim_math only
sim_damping_<name>  one per damping model: links sim_math only
sim_integrator_<name>  one per integrator: links sim_math only, generic over the System concept, never sees an energy
sim_solid           ElasticSystem (tet assembly): links sim_math and the energy modules it is built with
sim_strand          DER energies and DerSystem: links sim_math
sim                 World + SceneDesc: links everything above and turns a description into systems + integrators
```

A CMake check (`Tests/ModuleBoundaries.cmake`) reads each target's `LINK_LIBRARIES` and fails configuration if an energy links an integrator or another energy, or an integrator links anything but `sim_math`.

**Note (2026-11-26):** the modules as built in `Lib/CMakeLists.txt` differ from this sketch.
- There is no `sim_math` target. `LibMath.h` and the concept headers are header-only, and each module links Eigen only.
- The integrators are one module, `sim_integrators`, which links `sim_system` (the `System` concept and `Constraints`) and Eigen.
- `sim_energy_<name>` is one module per volume energy, made by `sim_volume_energy(<name> <File>)`, and `sim_volume_energies` is an interface target over all of them. `sim_energy_mass_spring`, `sim_energy_vertex_face_collision` and `sim_damping_green` are the other energy modules.
- `sim_strand` links `sim_system`, the mass-spring, collision and damping modules. `sim_solid` links `sim_system`, the collision module, `sim_volume_energies`, `sim_damping_green` and libigl.
- `Tests/ModuleBoundaries.cmake` was not written. Instead, `TestEnergies`, `TestElementEnergies`, `TestDamping`, `TestElasticSystem`, `TestIntegrators` and `TestDerSystem` link only the modules they test (`Tests/CMakeLists.txt`), so a module that needs more fails to link.

## Naming

HOBAK's ALL_CAPS types become this repo's style: `PascalCase` types, `camelCase` functions, everything in `namespace sim` (after P4b of the GLUT-removal plan). Examples: `VOLUME::HYPERELASTIC` → `sim::Energy` concept, `BACKWARD_EULER_POSITION` → `sim::BackwardEuler`, `MATRIX9` → `Mat9`.

## Target data shape

```cpp
namespace sim {

// Rotation-variant SVD (Irving et al. 2004 sign convention: det U = det V = 1, σ3 may be negative).
struct Svd { Mat3 u; Vec3 sigma; Mat3 v; };
auto RotationVariantSvd(const Mat3 &f) -> Svd;

// An isotropic or anisotropic hyperelastic energy density. Pure, no state beyond its parameters.
template <typename E>
concept Energy = requires(const E &e, const Mat3 &f, const Svd &svd) {
  { e.Psi(f) } -> std::same_as<Real>;
  { e.Pk1(f) } -> std::same_as<Mat3>;
  { e.Hessian(f) } -> std::same_as<Mat9>;          // ∂²Ψ/∂vec(F)²
  { e.ClampedHessian(f, svd) } -> std::same_as<Mat9>;  // analytic eigensystem, eigenvalues clamped ≥ 0
};

// The same four evaluations for energies over N stacked vertex DOFs instead of F: springs today,
// collision energies later.
template <typename E, int N>
concept ElementEnergy = requires(const E &e, const VecN<N> &x) {
  { e.Psi(x) } -> std::same_as<Real>;
  { e.Gradient(x) } -> std::same_as<VecN<N>>;
  { e.Hessian(x) } -> std::same_as<MatN<N>>;
  { e.ClampedHessian(x) } -> std::same_as<MatN<N>>;
};

struct Lame { Real mu, lambda; static auto FromYoungs(Real e, Real nu) -> Lame; };
struct StableNeoHookean { Lame lame; /* Psi, Pk1, Hessian, ClampedHessian */ };
struct StVK { Lame lame; ... };
// ...one struct per energy; Material = std::variant<StableNeoHookean, StVK, ...>

// Damping energy density over (F, Ḟ).
template <typename D>
concept DampingEnergy = requires(const D &d, const Mat3 &f, const Mat3 &fdot) {
  { d.Psi(f, fdot) } -> std::same_as<Real>;
  { d.Pk1(f, fdot) } -> std::same_as<Mat3>;
  { d.Hessian(f, fdot) } -> std::same_as<Mat9>;    // ∂²Ψ/∂vec(Ḟ)²
};
struct GreenDamping { Real mu; ... };
using Damping = std::variant<std::monostate, GreenDamping, Rayleigh>;

// What an integrator needs from a body: nothing about energies or meshes.
template <typename S>
concept System = requires(const S &s, const Vec &x, const Vec &v) {
  { s.Dofs() } -> std::same_as<int>;
  { s.Mass() } -> std::same_as<const SparseMat &>;
  { s.Forces(x, v) } -> std::same_as<Vec>;
  { s.Stiffness(x, true) } -> std::same_as<SparseMat>;   // clamped = true gives an SPD matrix
  { s.DampingMatrix(x) } -> std::same_as<SparseMat>;
  { s.Constraints() } -> std::same_as<const Constraints &>;
};

// Global assembly over one TetMesh: the one place element quantities scatter to DOFs. Satisfies System.
struct ElasticSystem {
  const TetMesh &mesh; const Material &material; const Damping &damping;
  auto Forces(const Vec &x, const Vec &v) const -> Vec;               // −∂E/∂x + damping
  auto Stiffness(const Vec &x, bool clamped) const -> SparseMat;      // Σ dFdxᵀ H dFdx, 12×12 per tet
  auto DampingMatrix(const Vec &x) const -> SparseMat;
};

// Constraints are a projection S = I − Σ nnᵀ over pinned and kinematic DOFs (Baraff & Witkin 1998).
struct Constraints { std::vector<int> pinned; /* kinematic targets later */ auto Filter() const -> SparseMat; };

struct ForwardEuler {};                                  // today's behaviour, kept as the explicit option
struct BackwardEuler { int maxNewtonIterations = 10; Real tolerance = 1e-6; };
struct Newmark { Real beta = 0.25, gamma = 0.5; };
struct Bdf2 {};
struct Quasistatic { int maxNewtonIterations = 50; };
using Integrator = std::variant<ForwardEuler, BackwardEuler, Newmark, Bdf2, Quasistatic>;
}
```

`SoftBody` and `Strand` in `SceneDesc` gain required `Integrator integrator` and `Damping damping` fields, with no default member initializers. Scenes are declarative, so every body states how it is integrated, and leaving it out is a compile error. `World` builds a `System` per body (`ElasticSystem` for soft bodies, `DerSystem` for strands) and steps it with the integrator the body names, through `std::visit`. Integrators are templates over `System`, so strands and volumes share them.

Implicit solves use Eigen `ConjugateGradient` on the clamped stiffness. Clamping makes it SPD, so CG is valid there, unlike in the deleted tridiagonal helper. They also run single-threaded in a fixed order, so lockstep peers on the same binary and ISA stay bit-identical (the voxel game's requirement).

## PR order and dependencies

```
M0 (current branch: GLUT removal + DER fixes + paper stiffness + P4b names) ──► H1 ──► H2 ──► H3 ──┬─► H4
                                                                                                    ├─► H5
                                                                                                    └─► E1..E6 (independent, each after H3)
```

Each PR branches from trunk after the one before it lands. E1 through E6 touch disjoint energy files plus one catalog line each, so they can land in parallel once H3 is in.

Baseline for every perf box: measure on trunk before the PR. The metric is ms per step for `bunny-drop` (Release, `TestSceneGolden`-style headless loop, median of 5 runs of 600 steps). A PR fails perf if its median regresses more than 10% with no reason given in the PR.

---

### M0. Land the current branch (prerequisite, plan `2026-11-26-glut-removal-scene-api.md`)
- [x] The DER math fixes are merged. Strand-drop runs past step 30 without exploding, which is the operator's report, and the goldens are regenerated and finite.
- [x] P4b (namespace `sim`, `SIM_` macros, target `sim`) and P5 docs are done.
- [x] Mass-spring Hessian (operator request, 2026-11-26). `MassSpring` becomes its own module, `sim_energy_mass_spring`, satisfying an `ElementEnergy<6>` concept over one edge's endpoints: `Psi(x)`, `Gradient(x)`, `Hessian(x)` and `ClampedHessian(x)`. The clamped Hessian uses the closed-form eigensystem. The axial eigenvalue is the stiffness along the edge. The two transverse eigenvalues are tension divided by length and go negative under compression, so they clamp to 0. The energy and force stay exactly as today, including the audit's per-edge rest lengths.
  - Verify: the gradient matches FD of Psi and the Hessian matches FD of the gradient, both ≤1e-6 relative. The clamped Hessian is PSD on random stretched and compressed edges. The unclamped Hessian has a negative eigenvalue under compression; that's the control. Strand goldens are unchanged bit for bit, because forces don't change.
  - **Note (2026-11-26):** strand-bend was the exception to "bit for bit" (833642a). Its golden moved from step 1500, by at most 2.2e-3. The old spring force used the edge length cached before the script moved a pin, and the module uses the current length. Rebuilding the old formula with the stale length reproduced the old golden byte for byte, and with the current length the new one. bunny-drop, strand-drop and strand-twist stayed bit-identical.
- [x] DER bending stiffness switched to the paper's per-vertex 2α/l̄_i, with α = 0.1 and l̄_i = |ē^{i-1}| + |ē^i| (operator decision 2026-11-26, audit finding 11). Goldens regenerated once after this and the DER fixes.
- [ ] Verify: everything in that plan's boxes, plus `strand-drop` in `sim_viewer` played for 20,000 steps with no NaN (Stats shows a finite step count, and the screenshot shows strands hanging under the comb).
- **Result (2026-11-26):** merged to `main` in PR #3 (`modernize-sim-core`). The DER fixes are 3f2cf43 (the bending force scatters all three kb blocks, so strand-bend no longer goes NaN), P4b is 9067ea9, the P5 docs are 3029f0e, the mass-spring module is 833642a and the paper's bending stiffness is 76c97e1. `TestSceneGolden` runs strand-drop finite through step 20500 headless. The last box stays open, because no commit records the 20,000-step viewer run or its screenshot.

### H1. Energy concept, SVD, and the test suite every energy must pass
- **Depends on:** M0.
- **Files:** new `Lib/Energy/Volume/Energy.h` (concept, `Svd`, `Lame`), `Lib/Energy/Volume/StableNeoHookean.{h,cc}` (port of `SNH`), `Lib/Energy/Volume/StVK.{h,cc}` (real Hessian; today's is an empty stub), `Lib/LibMath.h` (`RotationVariantSvd`), `Tests/TestEnergies.cc` (typed test over every `Material` alternative). Delete `HyperelasticMaterial.h`, `SNH.*`, `STVK.*`, `TestSNH.cc`.
- **Change:** energies become value types that satisfy `sim::Energy`. `TetMesh::ComputeMaterialForces` takes a `Material` and visits it.
- **Result you see:** one typed test suite that every new energy gets for free.
- **Unit:**
  - Pk1 matches FD of Psi, and Hessian matches FD of Pk1, to ≤1e-6 relative.
  - ClampedHessian is PSD (min eigenvalue ≥ −1e-10) on 1,000 random F, including inverted ones (det F < 0).
  - Ψ(R) = 0 and P(R) = 0 for random rotations R.
  - Ψ(RF) = Ψ(F) for isotropic energies.
  - RotationVariantSvd reconstructs F to 1e-12, with det U = det V = 1.
- **Live:** `bunny-drop` golden still passes bit for bit. The SNH force formula is unchanged, so any diff is a bug.
- **Perf:** bunny ms/step within 10% of trunk.
- **Result (2026-11-26):** branch `h1a-energy-concept` (landed as `land-h1a` and merged into `wave2-base`), then `h1b-wire-material` on top. `TestEnergies` runs StableNeoHookean and StVenantKirchhoff on 1,000 random F, about half inverted (b9224f6). After `Material` became a variant and the legacy classes were deleted, bunny-drop's positions after 600 steps hash identically, and its golden passes unregenerated (e06b0a1). StVK landed as `Lib/Energy/Volume/StVenantKirchhoff.{h,cc}`. No commit records the perf number.

### H2. ElasticSystem: assembly of forces, stiffness, damping, mass
- **Depends on:** H1.
- **Files:** new `Lib/Solid/ElasticSystem.{h,cc}`, `Lib/Solid/Constraints.{h,cc}`. `TetMesh` loses `ComputeMaterialForces` (moves here) and keeps geometry (Dm⁻¹, ∂F/∂x, volumes, mass). `Tests/TestElasticSystem.cc`.
- **Change:** the one place element terms scatter into global vectors and matrices. Stiffness assembly uses the existing ∂F/∂x (9×12) per tet.
- **Unit:**
  - `Stiffness(x)` equals FD of `Forces(x)` to ≤1e-6 relative on the bunny.
  - Stiffness is symmetric to 1e-12.
  - Clamped stiffness is PSD (smallest eigenvalue from Eigen `SelfAdjointEigenSolver` on a small mesh).
  - Filter zeroes pinned DOFs and is idempotent (S² = S).
- **Live:** `ForwardEuler` re-expressed over `ElasticSystem` reproduces the `bunny-drop` golden bit for bit.
- **Perf:** bunny ms/step within 10%. Stiffness assembly time is reported separately (new number, no gate).
- **Result (2026-11-26):** branch `h2-elastic-system`. On the bunny (710 tets) at a random deformation, `Stiffness` matches central FD of `Forces` to 7.5e-11 (SNH) and 9.2e-11 (StVK) relative, and is symmetric to 1.5e-14. The clamped matrix is PSD, and the unclamped one has eigenvalue -86.5 (SNH) and -0.98 (StVK) (70b7971). ForwardEulerVolume over `ElasticSystem` kept bunny-drop bit-identical, with FNV-1a 8edf3548a3b6b2d7 after 600 steps and bce8487b12ae57a8 after 6000. Median ms/step over 5 interleaved 6000-step runs fell from 0.0460 to 0.0294 (19f5107). No commit reports stiffness assembly time separately.

### H3. Time integrators
- **Depends on:** H2.
- **Files:** new `Lib/Solid/Integrators.{h,cc}` (one stepping function per alternative), `Lib/Sim/Scene.h` (`SoftBody::integrator`, `SoftBody::damping`), `Lib/Sim/World.cc`, `SceneCatalog/Catalog.cc` (add `bunny-drop-implicit`), `Tests/TestIntegrators.cc`. Delete `ForwardEulerVolume.h`, `BackwardEulerVolume.h`, `VolumeIntegrator.h`.
- **Change:** `ForwardEuler`, `BackwardEuler` (Newton on position with line search), `Newmark`, `Bdf2` and `Quasistatic` as `Integrator` alternatives.
- **Result you see:** the bunny runs at dt = 1/60 under Backward Euler and stays stable, where Forward Euler needs 1/3000.
- **Unit:** each check is behavioral and runs on a single tet and on the bunny.
  - BackwardEuler at dt = 1/60 for 600 steps stays finite, and its total energy is non-increasing (it's dissipative).
  - ForwardEuler at the same dt blows up. That's the control, so the test can tell stable from unstable.
  - Newmark (β = 0.25, γ = 0.5), undamped, keeps total energy within 1% over 600 steps.
    - **Note (2026-11-26):** the bunny runs this check at dt 1/240, and the tet at 1/60. The trapezoidal rule's energy error grows with how unresolved the stiffest modes are, and the bunny's small tets put its spectrum far past 1/dt at 1/60. The bunny drifted 2.2% at 1/60, then 0.53%, 0.18% and 0.054% as dt halved (`Tests/TestIntegrators.cc`, `NewmarkConservesEnergy`). At 37c9496 the tet drifts 0.26% at 1/60 and the bunny 0.18% at 1/240.
  - Bdf2 is second-order. Halving dt cuts the error against a dt/16 reference by about 4x, asserted as a ratio between 3 and 5.
    - **Note (2026-11-26):** the check ramps gravity on with (1 - cos(π t / T)) / 2 over the run. A step load excites every mode, and BDF2's error on the modes dt leaves unresolved converges more slowly. With a step load the bunny's ratio was 2.84 at 1/60, 2.90 at 1/120 and 3.22 at 1/240. With the ramp at 37c9496, the ratio is 3.96 (tet) and 4.20 (bunny) under Bdf2, against 2.13 and 2.15 under BackwardEuler.
  - Quasistatic: under gravity with pinned top vertices, the force residual drops below 1e-6, and the result matches BackwardEuler run to rest.
  - Newton converges within maxNewtonIterations on every step. If it doesn't, the step reports failure instead of returning garbage.
    - **Note (2026-11-26):** the default cap of 10 stays, but the tests and scenes raise it. Where clamping is active, the clamped stiffness makes Newton converge linearly. The accuracy tests run 100 iterations at tolerance 1e-10, and the bunny under BackwardEuler at 1/60 takes 15 at worst. bunny-drop-implicit and the scenes built from it set `maxNewtonIterations = 25`. The worst step took 17 at H3 (5d7aa1e) and 14 after C1b (3305ccd).
- **Live:** `sim_viewer` with `bunny-drop-implicit`, played 600 steps (10 s simulated). Screenshot shows the bunny hanging and deformed, with Stats finite. Compare the screenshot with the forward-Euler bunny at the same simulated time.
- **Perf:** report ms per simulated second for the implicit bunny vs the explicit one. That's the number that matters for a game. It's new, so there's no gate, but it gets explained per the benchmark checklist.
- **Result (2026-11-26):** branch `h3-integrators`, with the energy catalog merged in through `wave-h3e`. The five integrators live in `Lib/Solid/Integrators.{h,cc}` (module `sim_integrators`), and `SoftBody` gained a required `integrator` (51b9337). bunny-drop names ForwardEuler and stays bit-identical by FNV-1a hash after 600 and 14000 steps. At 37c9496, BackwardEuler at 1/60 runs 600 steps finite, and the bunny's worst one-step energy rise is 0. ForwardEuler at 1/60 blows up at step 4 (tet) and step 5 (bunny). Quasistatic lands within 4.1e-5 of BackwardEuler at rest, on a bunny displacement of 0.53. bunny-drop-implicit runs at dt 1/20, larger than the 1/60 above. It stays finite, never sinks past its epsilon, and rests past the ramp's foot (5d7aa1e). No commit records ms per simulated second. The worst 20 Hz tick took 38.3 to 38.5 ms over 5 runs after C1b (3305ccd). No commit records a viewer screenshot.

### H4. Green damping
- **Depends on:** H3.
- **Files:** `Lib/Energy/Volume/GreenDamping.{h,cc}`, a `Damping` alternative, `Tests/TestEnergies.cc` (damping section).
- **Unit:**
  - Pk1 matches FD of Ψ with respect to Ḟ.
  - Damping force is zero under rigid motion (Ḟ = ωF with ω skew).
  - Energy decays monotonically in a damped BackwardEuler run.
- **Live:** `bunny-drop-implicit` with damping visibly settles faster. Report steps until max velocity < 1e-3, damped vs undamped.
- **Result (2026-11-26):** branch `h4-damping`. `GreenDamping` landed as its own module, `sim_damping_green`, in `Lib/Energy/Damping/`, with its tests in `Tests/TestDamping.cc` on 1,000 random (F, Ḟ) (cc3b626). `Damping` is `NoDamping`, `GreenDamping` or `Rayleigh`, and `SoftBody::damping` is required (19fdc82). bunny-drop-implicit-damped (`GreenDamping{.mu = 2}`) keeps its max vertex speed below 1e-3 from step 275 (13.75 s). Undamped, the bunny stays above it until step 370 (18.5 s). Newton's worst step falls from 14 iterations to 7 (15b28d0). At 37c9496 under BackwardEuler, the damped energy falls on every step, from 0.00395 to 3.5e-8 on the tet and from 0.00304 to 8.7e-9 on the bunny.
- **Note (2026-11-26):** H3's Newton held the damping matrix at the step's first guess. `GreenDamping` and Rayleigh's stiffness term depend on x, so the residual stopped being the gradient of the line-search potential, and Newton stalled. Each iterate now line-searches on a potential with the damping matrix evaluated at that iterate (b3932ab). The damped runs stop Newton at 1e-8, not 1e-10. Near rest the line search cannot certify a 1e-10 step, because Psi's round-off exceeds the band 1e-10 |Phi|.
- **Note (2026-11-26):** the Newton Jacobian still leaves out how D(x) v changes with x, so Newton converges linearly under material damping (`Lib/Solid/Integrators.h`). A fix for the Newton objective under x-dependent damping is in progress on branch `newton-damping-objective`. It is not on this branch.

### H5. Strands on the shared integrators
- **Depends on:** H3.
- **Files:** `Lib/Strand/DerSystem.{h,cc}` (satisfies `System`), the DER Hessian (bending and twist; the inextensibility springs reuse M0's `ClampedHessian`) in `Lib/Energy/Strand/`, `Lib/Sim/World.cc`, `SceneCatalog/Catalog.cc` (strand scenes now name their integrator), and `Tests/TestDerSystem.cc`. Delete `ForwardEulerStrand.h` and `StrandIntegrator.h`. Strand `ForwardEuler` becomes the generic integrator plus the existing velocity filter, expressed as `Damping` on the strand.
- **Unit:**
  - DER Hessian matches FD of the forces to ≤1e-6 relative.
  - The clamped Hessian is PSD, with bending and twist blocks each projected.
  - Strand-twist under the generic ForwardEuler reproduces the post-M0 golden bit for bit.
    - **Note (2026-11-26):** strand-twist moved 9.2e-15 from the damping's rounding, inside its golden's 1e-12 tolerance. The velocity filter became Rayleigh mass damping c / m, so strand-drop's damping drop to 0.45 moved one step later (738117e). Two goldens changed on purpose:
      - strand-bend, max diff 4.1e-5 (738117e). The old integrator read bending and twist from the curvature it cached before `MovePinned` moved the pins, and read springs from the moved vertices. `Forces(x)` is now a function of x alone. A base build patched with only that fix and the Rayleigh form of the filter hashes bit-identically to the new code on strand-bend (7e6d49f5f908fae0) and strand-twist (08332e589715fdd7).
      - strand-drop, max diff 0.167 at step 20000 (e6eab54). Collider triangles wind counterclockwise seen from outside, and the old strand path stacked them as found, so the contact energy pulled over-face vertices toward the comb's inside. `DerSystem` now reverses the triangle, as `ElasticSystem` does. Over the full 30000 steps the closest approach to the comb went from 0.032 to 0.090.
- **Live:** strand-drop under BackwardEuler at dt = 1/60 runs 600 steps finite in `sim_viewer`, with a screenshot.
- **Perf:** ms per simulated second, explicit vs implicit strand-drop.
- **Result (2026-11-26):** branch `h5-strands-shared-integrators`. `DerSystem` (`Lib/Strand/`, module `sim_strand`) is one strand as a `System`, and `Strand` gained required `integrator` and `damping` fields. `ForwardEulerStrand.h` and `StrandIntegrator.h` are deleted (738117e). strand-drop-implicit runs the hair under BackwardEuler at dt 1/60. Over 600 steps every strand-step converges. Newton takes 2.03 iterations on average and 3 at worst, with 134 CG iterations per strand-step. At t = 10 s it is within 6.9e-3 of the explicit run, on strands 7.75 long (7c3e515). The check is headless, in `TestWorld` and a new golden. No commit records a viewer screenshot or the perf number.

### E1 to E6. More solid-mechanics energies (one PR each, after H3)
Each one adds a struct satisfying `sim::Energy`, a `Material` alternative, a line in the H1 typed suite (so it gets FD, PSD, rest-state and rotation tests automatically), and a catalog scene that swaps the bunny's material.
- [x] **E1 Linear elasticity** (small-strain, for reference and validation).
- [x] **E2 Neo-Hookean (Bonet & Wood)**, with an inversion guard tested on det F → 0⁺.
- [x] **E3 ARAP**, with analytic eigensystem clamping (Smith 2019).
- [x] **E4 Symmetric Dirichlet**.
- [x] **E5 Anisotropic fiber energies** (anisotropic ARAP, anisotropic StVK, Fung). `SoftBody` gains a per-tet fiber direction (`std::optional<Vec3>` uniform fiber, or per-tet field). The invariant test becomes "Ψ invariant to rotations about the fiber".
- [x] **E6 SNH with barrier** (inversion barrier).
- **Verify each:**
  - **Unit:** the typed suite passes. Energy-specific known values are checked against a hand derivation for one F.
  - **Live:** the catalog scene runs 600 implicit steps finite, with a screenshot in the PR.
  - **Perf:** ms/step vs SNH on the same scene.
- **Result (2026-11-26):** the energies are on branch `e-energy-catalog` (49f2652 to 43d3882), and the fiber term and scenes are on `e-catalog-scenes` (74c624f, 37c9496). Each energy is its own module, joined through `sim_volume_energies`, so a new energy is one CMake line. `TestEnergies` runs all ten types. Each catalog scene is bunny-drop-implicit-damped with its material swapped, under BackwardEuler at dt 1/20. `TestWorld` checks that each stays finite, converges on every step, never passes the ramp or floor by its epsilon, and rests past the ramp's foot. A new golden pins each. bunny-snh-barrier matches bunny-drop-implicit-damped to 2e-11, because no tet comes below J = 0.55. Each fiber moves the bunny's vertices at most 0.023. No commit records a screenshot or the ms/step comparison.
- **Note (2026-11-26):** these thresholds changed during execution:
  - The scenes run 280 implicit steps (14 s at dt 1/20), not 600. That is the length of the ramp scene they share.
  - Two scenes raise Green damping above the base bunny's mu = 2. StVK has zero energy at every reflection. At mu = 2 about 30 tets invert on impact and Newton fails to converge on step 180, while mu = 8 keeps every tet uninverted. ARAP has no volume term. At mu = 2 tets flip and the bunny still moves 5.0e-3 over the last second, while mu = 4 settles it to 1.6e-3 (37c9496).
  - The suite samples each energy on its own domain. Symmetric Dirichlet uses every |s_i| ≥ 0.1, ARAP uses every pair of singular values summing to at least 0.1, anisotropic ARAP uses |I4| ≥ 0.1, and the SNH barrier uses J ≥ 0.01.
  - Linear elasticity is not rotation invariant, so the suite asserts that rotations strain it and that Psi(R F) differs from Psi(F) on at least 900 of 1,000 samples (49f2652). The fiber energies are invariant to left rotations and to right rotations about the fiber, and an arbitrary right rotation must change Psi on at least 900 of 1,000 samples (90ec88c).
  - The symmetry test's tolerance gained an absolute floor of 1e-15 times the modulus. Near the rest length sqrt(I5) - 1 cancels, and one anisotropic ARAP sample differed by 2.7e-12 relative under a left rotation (724afc0).
  - The fiber is not per tet. It is a uniform, body-local `SoftBody::fiber` (a `FiberMaterial`) that adds to the isotropic `Material` in every tet. A fiber energy alone collapses a body, so it is never the whole material (74c624f).

## Review gates
- H3 and H5 change how scenes are authored (bodies gain required fields), so they wait for the operator's review before merge.
- Every PR's description includes a "no HOBAK code" statement, and the reviewer checks it.

### S1. Place bodies by centroid (operator request, 2026-11-26)
- **Depends on:** M0. Independent of H1 to H3, so it can land first.
- **Why:** bodies are authored in raw world coordinates today. `Strand::points` is a world-space matrix, `SoftBody::pinRegion` is a world-space box, and `Collider` only takes a translation offset. Moving a body means rewriting all of its geometry.
- **Change:** every body is a shape plus a placement.
  ```cpp
  struct Placement { Vec3<Real> centroid = Vec3<Real>::Zero(); Eigen::Quaternion<Real> rotation = Eigen::Quaternion<Real>::Identity(); Real scale = 1; };
  struct MeshFile { fs::path path; };
  struct Box { Vec3<Real> halfExtents; };            // colliders: floor, ramp, walls
  struct SoftBody { MeshFile mesh; Placement place; Material material; std::optional<Eigen::AlignedBox<Real,3>> pinRegion; /* body-local */ };
  struct Collider { std::variant<MeshFile, Box> shape; Placement place; };
  struct Strand { Mat<Real> points; Placement place; ... };   // points are body-local
  ```
  - `World` recenters each rest shape on its centroid (volume-weighted for tet meshes, area-weighted for collider surfaces, vertex mean for strands), then applies scale, rotation and translation.
  - Pin regions and `MoveCollider` paths are body-local, so moving a body doesn't break them.
  - `MoveCollider` moves `place.centroid`.
- **Unit:** a placed body's rest centroid equals `place.centroid` to 1e-12, rotation keeps edge lengths and tet volumes to 1e-12, and identity placement reproduces every current golden bit for bit.
- **Result (2026-11-26):** branch `s1-centroid-placement`, landed as `land-s1`. A placed tet mesh, strand and `Box` rest on `place.centroid` to 1e-12. Rotation and scale keep edge lengths and tet volumes, a pin region moves with its body, `Box` faces wind outward, and a non-positive scale is rejected (10158d8). Every golden passes unregenerated at 1e-12.
- **Note (2026-11-26):** "bit for bit" did not hold for every scene. Each catalog `place.centroid` is the body's old world centroid, and recentering then moving back rounds differently. bunny-drop moved by at most 1.6e-15 and strand-drop by at most 2.7e-14 absolute over the golden runs, inside the goldens' 1e-12 tolerance (7cc17fe).

### C1. Soft bodies collide with static colliders (operator request, 2026-11-26)
- **Depends on:** H2, because contact forces and stiffness are assembled in `ElasticSystem` so explicit and implicit integrators both get them.
- **Today:** only strand vertices collide, through `VertexFaceSqrtCollision`. That type has `Gradient` only. It has no `Psi` or Hessian, it isn't an `ElementEnergy`, and nothing tests it.
- **Change:**
  - Port `VertexFaceSqrtCollision` to `ElementEnergy<12>` with Psi, Gradient, Hessian and ClampedHessian, derived from the published math.
  - Detect soft-body surface vertices against collider triangles each step, the same way strands do.
  - Add the contact terms to `ElasticSystem::Forces` and `Stiffness`.
  - Contact is frictionless, so the bunny slides. Friction is a later, separate energy.
- **Scene:** `bunny-drop` becomes unpinned. The bunny starts above a tilted `Box` ramp, slides down it and settles on a `Box` floor. The golden is regenerated with that reason.
- **Unit:**
  - The collision energy passes the `ElementEnergy` FD suite (gradient vs FD of Psi, Hessian vs FD of gradient, ≤1e-6 relative) and the PSD clamp test.
  - A single tet dropped on a floor comes to rest with no vertex below the floor minus the envelope.
  - On a frictionless ramp, the tet's centroid acceleration along the slope matches g·sin θ within 2%.
- **Live:** `just demo` shows the bunny falling, sliding down the ramp and resting on the floor. Stats stay finite, and the bunny never passes through either collider.
- **Result (2026-11-26):** branch `c1-soft-contact`. `VertexFaceSqrtCollision` is an `ElementEnergy<12>` in its own module, `sim_energy_vertex_face_collision`, and passes the typed suite plus 300 random contacts across the face, edge and corner regions (750bd61). A tilted tet dropped on a floor rests with max speed 5e-5 and never sinks below the floor's top. A face-down tet on a frictionless 30 degree ramp accelerates at 0.99999 of g sin θ (ae3ae31). bunny-drop is unpinned. It falls onto a `Box` ramp, slides down it, and rests on a `Box` floor with its centroid 0.79 past the ramp's foot after 14 s. No vertex enters either box (deepest -0.0065), and the bunny moves 1 mm over its last second. Its golden was regenerated for the new scene (6ed4700). The check is headless, in `TestWorld`. No commit records the `just demo` run.
- **Note (2026-11-26):** contact needs damping to stop a slide on a frictionless floor, so `SoftBody` gained `collisionDamping` along the triangle normal and mass-proportional `drag` (bf85712). bunny-drop uses `drag = 0.75`.

### C1b. Continuous contact off the face (2026-11-26)
- **Result (2026-11-26):** branch `c1b-contact-continuity`, after H3. Contacts are found once per step and held through Newton, so a vertex can slide off its triangle and cross the triangle's plane beside it. The old energy jumped there, by 0.030 on the new test's path. It now stays continuous everywhere except on the triangle's boundary (4bffb39). bunny-drop-implicit went back from collisionEpsilon 0.03 to the default 0.02. Every step converges, with Newton at most 14 and 3.50 on average. The worst 20 Hz tick fell from 50.4 to 50.6 ms at 0.03 to 38.3 to 38.5 ms (5 runs each, 3305ccd).
- **Note (2026-11-26):** this is a bit-identity exception. strand-drop's golden moved by at most 3.2e-1 from step 5000, because almost all strand contacts sit beside the face and behind it in the energy's front convention, which is the case the old energy flipped. bunny-drop-implicit moved by 4.0e-8 before the epsilon change. bunny-drop still matches its golden to 1e-12.

## Open calls (operator)
1. **Volume collisions.** Static-collider contact for soft bodies is now C1 (operator, 2026-11-26). Edge-edge contact, self-collision and friction remain out of scope. The voxel game wants a static ray query against voxel chunks first.
Settled 2026-11-26: there is no default integrator, because every body declares one. Strands on the shared integrators are in scope (H5). Bending stiffness follows the paper (M0).

## Unproven until executed
- **H3's ratios.** The order-of-accuracy and energy-drift thresholds (BDF2 ratio between 3 and 5, Newmark within 1%) are expected values, not yet measured on this mesh. If one fails, the PR has to explain why before anyone relaxes the threshold.
- **Implicit cost.** The ms per simulated second for the implicit bunny is unknown. If PCG iterations dominate, a preconditioner choice becomes its own follow-up.
- **Note (2026-11-26):** both H3 ratios are now measured, and both passed only after the changes noted under H3. Newmark on the bunny runs at dt 1/240, and the BDF2 check ramps gravity on. The implicit bunny's ms per simulated second is still unmeasured. The worst 20 Hz tick of bunny-drop-implicit is 38.3 to 38.5 ms (3305ccd), and CG took at most 2586 iterations on one step.
