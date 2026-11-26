# Plan: GL-free sim core, declarative scenes, GLFW/OpenGL/ImGui viewer

## Goal

Turn sim-base into a physics library a game can embed. The core has no windowing or GL. Scenes are described as data instead of hand-built inside GLUT driver files. A separate viewer renders any scene with modern OpenGL and ImGui.

## Where the code is today (measured)

- Six GLUT executables (`main.cc`, `Scenes/*.cc`, 227 to 330 lines each) each repeat about 150 lines of camera, input, and GLUT boilerplate.
- Scene setup and scripting are spread across three places. Some is in `Lib/Scenes/*` constructors, some in driver `main()`, and some in `GlutIdle` step-counter branches (`gSteps > 1000`, `gSteps > 2000`, `frame > 400`, `sceneFrames > 200`).
- `frame` and `sceneFrames` advance inside the *render* path (every 50 steps). So scripted physics depends on redraw cadence.
- Rendering is baked into the physics types (`TetMesh::Draw`, `StrandMesh::Draw`, `CollisionMesh::Draw`, `LibGui`). `base` links GLUT and OpenGL, so nothing can embed it without a window.
- `Lib/Scenes/StrandTwist.*` is an empty shell. `BackwardEulerVolume::Step` and `STVK::Hessian` are stubs.
- OBJ export writes to a hardcoded `/Users/jarredparr/Downloads/output`.

## Target shape

Three CMake targets with one-way dependencies: `sim` ← `sim_scenes` ← `sim_viewer`. A game links `sim` alone (or with `sim_scenes`).

### 1. `sim` (core library, no GL, no GLFW)

Everything now in `Lib/` except rendering. All `Draw()` methods, `LibGui`, `OpenGL.h`, and `Camera.h` leave the core.

Scene description is plain value types:

```cpp
namespace sim {
struct Strand    { Mat<Real> points; std::vector<int> pinned; Real collisionStiffness = 10, collisionEpsilon = 0.2; };
struct SoftBody  { fs::path mesh; Material material; Eigen::AlignedBox3d pinRegion; };   // Material = variant<SNH, STVK>
struct Collider  { fs::path mesh; Vec3<Real> offset; };
using Body = std::variant<Strand, SoftBody, Collider>;

struct BodyId { int index; };   // a typed handle, not a bare int

// Scripted behaviour is data on a step window instead of if-branches in an idle callback.
struct MovePinned   { BodyId body; int vertex; Real fromX, toX; };      // StrandBend
struct MoveCollider { BodyId body; Vec3<Real> from, to; };              // comb sweep in StrandDrop
struct SetTwist     { BodyId body; Real left, right; };                 // StrandTwist
struct SetDamping   { BodyId body; Real coefficient; };                 // the old `bump` flag (0.75 -> 0.45)
struct Action { int firstStep, lastStep; std::variant<MovePinned, MoveCollider, SetTwist, SetDamping> effect; };

struct SceneDesc {
  std::string name;
  Real dt;
  Vec3<Real> gravity{0, -0.981, 0};
  std::vector<Body> bodies;
  std::vector<Action> script;
  std::optional<int> stopAtStep;
  std::optional<CameraHint> view;   // eye/lookAt/up. Data only; the viewer reads it.
};

class World {
public:
  explicit World(const SceneDesc &);      // the only place descriptions turn into meshes and integrators
  void Step();                            // applies due actions, collisions, then integrators
  void SetExtraAcceleration(Vec3<Real>);  // interactive pulls (arrow keys today)
  int step() const;  bool finished() const;
  std::span<const BodyState> bodies() const;  // read-only positions, edges/faces, and collision contacts for renderers
};
}
```

Every script trigger becomes a function of `World::step()` alone. That fixes the render-cadence coupling.

### 2. `sim_scenes` (catalog)

One function per scene returning `SceneDesc`, plus a registry the viewer lists:

```cpp
struct SceneEntry { std::string_view name; SceneDesc (*make)(); };
std::span<const SceneEntry> Catalog();   // bunny-drop, strand-drop, strand-bend, strand-twist, frame-debug
```

Adding a scene means writing one function and one registry line. No driver file.

### 3. `sim_viewer` (one executable, replaces all six GLUT drivers)

- `Window`: GLFW window and OpenGL 4.1 core context (the macOS ceiling), via glad. Translates GLFW input into viewer events.
- `Camera`: moved from `Lib/Camera.h` unchanged.
- `Renderer`: two shader programs, unlit lines/points and lit triangles, with dynamic VBOs refilled from `World::bodies()` each frame. Also draws the grid and axes.
- `ui/`: each ImGui panel is a free function over a shared `ViewerState&` and can be added or removed independently. Panels are scene picker, playback (play, pause, step, reset, steps per frame), stats, body inspector, and OBJ export (output dir chosen in the UI or by `--export <dir>`).
- `App`: the frame loop. Poll input, advance `World` by the playback budget, render, draw panels.

Dependencies come from FetchContent: GLFW 3.4, current Dear ImGui (its GLFW and OpenGL3 backends), and glad from libigl's existing recipe.

### Shapes considered

- **A (chosen): value-type description plus a typed action timeline.** Scenes can be inspected, so ImGui can show bodies and the timeline. Invalid wiring shows up as a type error. `World` is the one place construction happens.
- **B (rejected): builder plus per-step `std::function` callbacks.** It's shorter to write. But callbacks capture ad hoc state, which brings back the per-scene step counters this plan deletes, and the UI can't inspect them.

New action kinds get added when a scene needs them. There is no speculative generic escape hatch.

## Phases

Each phase lands green before the next one starts.

### P0. Finish the warning sweep (in progress before this plan)
- [ ] Add virtual destructors to `CollisionMesh`, `StrandIntegrator`, and `VolumeIntegrator`. Fix the `DiscreteElasticRod` initializer order. Fix the duplicate gtest link.
- [ ] Verify: a clean rebuild with `-Wall -Wextra -Wpedantic` shows zero warnings in project code, and `ctest` passes.

### P1. Behavior pin (before any structure moves)
- [ ] `Tests/TestSceneGolden.cc` rebuilds each current scene setup exactly as the drivers do. It runs a fixed number of steps, including past every script trigger (bend at 1000, twist stop at 2000, the comb sweep). It writes final positions to `Tests/golden/<scene>.txt`, generated from pre-refactor code.
- [ ] Verify: the golden files are committed, and the test passes against itself on trunk.

### P2. `sim` core: SceneDesc, World, catalog
- [ ] Add the types above and `World`. Port the five scenes into `sim_scenes`. Delete the empty `StrandTwist` shell.
- [ ] Switch the golden test to `World(Catalog()[i].make())`.
- [ ] Verify: the golden test passes within 1e-12, meaning trajectories match the old drivers. Any intentional semantic change (the cadence fix) gets a regenerated golden and a written reason in the commit.

### P3. Viewer
- [ ] Add `sim_viewer` with Window, Renderer, Camera, `ui/` panels, and App.
- [ ] Verify: launch the viewer, run each catalog scene, and capture a screenshot of each. Check interaction by hand: orbit, pan, zoom, play, step, reset, scene switching, and export. Diff exported OBJs against the old `--save` output for one scene.

### P4. Delete the old world (one wave)
- [ ] Remove `main.cc`, `Scenes/*.cc`, every `Draw()` in `Lib/`, `LibGui.*`, `OpenGL.h`, and the GLUT/OpenGL link on the core.
- [ ] Verify: `sim` builds and links with no GL/GLUT in `otool -L` or the link line. A tiny headless `examples/embed.cc` steps a scene with no window. All tests pass.

### P5. Docs
- [ ] Update `CLAUDE.md` and `README.md` for the new targets, the scene-authoring flow, and the viewer. Point the mitsuba scripts at the export dir.

## Open calls (my defaults; the operator can override)
1. One viewer executable with a scene picker replaces per-scene binaries. A `--scene <name>` flag keeps one-shot launches.
2. OpenGL 4.1 core, because that's what macOS supports.
3. The stubs (`BackwardEulerVolume`, `STVK::Hessian`) stay out of scope. A scene that selects them gets a construction error instead of silent garbage.
