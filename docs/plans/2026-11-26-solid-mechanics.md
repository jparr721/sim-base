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

// Global assembly over one TetMesh: the one place element quantities scatter to DOFs.
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

`SoftBody` in `SceneDesc` gains `Integrator integrator` and `Damping damping`. `World` builds the `ElasticSystem` once and calls one stepping function per integrator through `std::visit`. Strands keep their own DER integrator. Running strands on the implicit integrators needs a DER Hessian, which is out of scope here (see Open calls).

Implicit solves use Eigen `ConjugateGradient` on the clamped stiffness. Clamping makes it SPD, so CG is valid there, unlike in the deleted tridiagonal helper. They also run single-threaded in a fixed order, so lockstep peers on the same binary and ISA stay bit-identical (the voxel game's requirement).

## PR order and dependencies

```
M0 (current branch: GLUT removal + DER fixes + P4b names) ──► H1 ──► H2 ──► H3 ──► H4
                                                                       └────────► E1..E6 (independent of each other, each after H3)
```

Each PR branches from trunk after the one before it lands. E1 through E6 touch disjoint energy files plus one catalog line each, so they can land in parallel once H3 is in.

Baseline for every perf box: measure on trunk before the PR. The metric is ms per step for `bunny-drop` (Release, `TestSceneGolden`-style headless loop, median of 5 runs of 600 steps). A PR fails perf if its median regresses more than 10% with no reason given in the PR.

---

### M0. Land the current branch (prerequisite, plan `2026-11-26-glut-removal-scene-api.md`)
- [ ] The DER math fixes are merged. Strand-drop runs past step 30 without exploding, which is the operator's report, and the goldens are regenerated and finite.
- [ ] P4b (namespace `sim`, `SIM_` macros, target `sim`) and P5 docs are done.
- [ ] Verify: everything in that plan's boxes, plus `strand-drop` in `sim_viewer` played for 20,000 steps with no NaN (Stats shows a finite step count, and the screenshot shows strands hanging under the comb).

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

### H3. Time integrators
- **Depends on:** H2.
- **Files:** new `Lib/Solid/Integrators.{h,cc}` (one stepping function per alternative), `Lib/Sim/Scene.h` (`SoftBody::integrator`, `SoftBody::damping`), `Lib/Sim/World.cc`, `SceneCatalog/Catalog.cc` (add `bunny-drop-implicit`), `Tests/TestIntegrators.cc`. Delete `ForwardEulerVolume.h`, `BackwardEulerVolume.h`, `VolumeIntegrator.h`.
- **Change:** `ForwardEuler`, `BackwardEuler` (Newton on position with line search), `Newmark`, `Bdf2` and `Quasistatic` as `Integrator` alternatives.
- **Result you see:** the bunny runs at dt = 1/60 under Backward Euler and stays stable, where Forward Euler needs 1/3000.
- **Unit:** each check is behavioral and runs on a single tet and on the bunny.
  - BackwardEuler at dt = 1/60 for 600 steps stays finite, and its total energy is non-increasing (it's dissipative).
  - ForwardEuler at the same dt blows up. That's the control, so the test can tell stable from unstable.
  - Newmark (β = 0.25, γ = 0.5), undamped, keeps total energy within 1% over 600 steps.
  - Bdf2 is second-order. Halving dt cuts the error against a dt/16 reference by about 4x, asserted as a ratio between 3 and 5.
  - Quasistatic: under gravity with pinned top vertices, the force residual drops below 1e-6, and the result matches BackwardEuler run to rest.
  - Newton converges within maxNewtonIterations on every step. If it doesn't, the step reports failure instead of returning garbage.
- **Live:** `sim_viewer` with `bunny-drop-implicit`, played 600 steps (10 s simulated). Screenshot shows the bunny hanging and deformed, with Stats finite. Compare the screenshot with the forward-Euler bunny at the same simulated time.
- **Perf:** report ms per simulated second for the implicit bunny vs the explicit one. That's the number that matters for a game. It's new, so there's no gate, but it gets explained per the benchmark checklist.

### H4. Green damping
- **Depends on:** H3.
- **Files:** `Lib/Energy/Volume/GreenDamping.{h,cc}`, a `Damping` alternative, `Tests/TestEnergies.cc` (damping section).
- **Unit:**
  - Pk1 matches FD of Ψ with respect to Ḟ.
  - Damping force is zero under rigid motion (Ḟ = ωF with ω skew).
  - Energy decays monotonically in a damped BackwardEuler run.
- **Live:** `bunny-drop-implicit` with damping visibly settles faster. Report steps until max velocity < 1e-3, damped vs undamped.

### E1 to E6. More solid-mechanics energies (one PR each, after H3)
Each one adds a struct satisfying `sim::Energy`, a `Material` alternative, a line in the H1 typed suite (so it gets FD, PSD, rest-state and rotation tests automatically), and a catalog scene that swaps the bunny's material.
- [ ] **E1 Linear elasticity** (small-strain, for reference and validation).
- [ ] **E2 Neo-Hookean (Bonet & Wood)**, with an inversion guard tested on det F → 0⁺.
- [ ] **E3 ARAP**, with analytic eigensystem clamping (Smith 2019).
- [ ] **E4 Symmetric Dirichlet**.
- [ ] **E5 Anisotropic fiber energies** (anisotropic ARAP, anisotropic StVK, Fung). `SoftBody` gains a per-tet fiber direction (`std::optional<Vec3>` uniform fiber, or per-tet field). The invariant test becomes "Ψ invariant to rotations about the fiber".
- [ ] **E6 SNH with barrier** (inversion barrier).
- **Verify each:**
  - **Unit:** the typed suite passes. Energy-specific known values are checked against a hand derivation for one F.
  - **Live:** the catalog scene runs 600 implicit steps finite, with a screenshot in the PR.
  - **Perf:** ms/step vs SNH on the same scene.

## Review gates
- H3 changes how scenes are authored (`SoftBody` gains fields), so it waits for the operator's review before merge.
- Every PR's description includes a "no HOBAK code" statement, and the reviewer checks it.

## Open calls (operator)
1. **Strands on implicit integrators.** That needs a correct DER Hessian (the old one was deleted in the audit as wrong). It's a separate plan if you want it.
2. **Volume collisions.** HOBAK's vertex-face and edge-edge collision energies (Kim's sqrt / hybrid) and self-collision are not in this plan. The voxel game wants a static ray query against voxel chunks first.
3. **Default integrator for new soft-body scenes.** I'd default to `BackwardEuler`, since games need large stable steps (Experience First). `ForwardEuler` stays available for validation.

## Unproven until executed
- **H3's ratios.** The order-of-accuracy and energy-drift thresholds (BDF2 ratio between 3 and 5, Newmark within 1%) are expected values, not yet measured on this mesh. If one fails, the PR has to explain why before anyone relaxes the threshold.
- **Implicit cost.** The ms per simulated second for the implicit bunny is unknown. If PCG iterations dominate, a preconditioner choice becomes its own follow-up.
