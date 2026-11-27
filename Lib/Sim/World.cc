#include "World.h"

#include <Solid/ElasticSystem.h>
#include <Solid/Integrators.h>
#include <Strand/DerSystem.h>
#include <igl/parallel_for.h>
#include <igl/readOBJ.h>

#include <algorithm>
#include <format>
#include <stdexcept>

namespace sim {

namespace {

template <class... Ts> struct Overloaded : Ts... {
  using Ts::operator()...;
};

[[noreturn]] void Malformed(const std::string &message) {
  throw std::invalid_argument(message);
}

auto PathSamples(const Action &action) -> int {
  if (!action.lastStep) {
    Malformed("a path action needs a lastStep");
  }
  const int span = *action.lastStep - action.firstStep + 1;
  if (span % action.holdSteps != 0) {
    Malformed(std::format("path action steps {}..{} are not a whole number of "
                          "{}-step holds",
                          action.firstStep, *action.lastStep, action.holdSteps));
  }
  return span / action.holdSteps;
}

auto StrictlyInside(const Eigen::AlignedBox<Real, 3> &box,
                    const Vec3<Real> &point) -> bool {
  return (point.array() > box.min().array()).all() &&
         (point.array() < box.max().array()).all();
}

void ValidatePlacement(const Placement &place) {
  if (!(place.scale > 0) || std::abs(place.rotation.norm() - 1) > 1e-12) {
    Malformed("a placement needs a positive scale and a unit rotation");
  }
}

// Rows of `local` scaled, rotated and moved so the origin lands on
// place.centroid.
auto Placed(const Placement &place, const Mat<Real> &local) -> Mat<Real> {
  const Mat3<Real> linear = place.scale * place.rotation.toRotationMatrix();
  return (local * linear.transpose()).rowwise() + place.centroid.transpose();
}

// A SoftBody's fiber, turned from body-local into world axes by its
// placement. The direction must be unit length.
auto PlacedFiber(const Placement &place, std::optional<FiberMaterial> fiber)
    -> std::optional<FiberMaterial> {
  if (fiber) {
    std::visit(
        [&](auto &energy) {
          if (!(std::abs(energy.fiber.norm() - 1) <= 1e-12)) {
            Malformed("a fiber direction must be unit length");
          }
          energy.fiber = place.rotation * energy.fiber;
        },
        *fiber);
  }
  return fiber;
}

auto Recentered(const Mat<Real> &v, const Vec3<Real> &centroid) -> Mat<Real> {
  return v.rowwise() - centroid.transpose();
}

auto VolumeCentroid(const Mat<Real> &v, const Mat<int> &t) -> Vec3<Real> {
  Vec3<Real> moment = Vec3<Real>::Zero();
  Real volume = 0;
  for (int ii = 0; ii < t.rows(); ++ii) {
    const Vec3<Real> a = v.row(t(ii, 0));
    const Vec3<Real> b = v.row(t(ii, 1));
    const Vec3<Real> c = v.row(t(ii, 2));
    const Vec3<Real> d = v.row(t(ii, 3));
    const Real tetVolume = (b - a).dot((c - a).cross(d - a)) / 6;
    moment += tetVolume * (a + b + c + d) / 4;
    volume += tetVolume;
  }
  return moment / volume;
}

auto AreaCentroid(const Mat<Real> &v, const Mat<int> &f) -> Vec3<Real> {
  Vec3<Real> moment = Vec3<Real>::Zero();
  Real area = 0;
  for (int ii = 0; ii < f.rows(); ++ii) {
    const Vec3<Real> a = v.row(f(ii, 0));
    const Vec3<Real> b = v.row(f(ii, 1));
    const Vec3<Real> c = v.row(f(ii, 2));
    const Real triangleArea = (b - a).cross(c - a).norm() / 2;
    moment += triangleArea * (a + b + c) / 3;
    area += triangleArea;
  }
  return moment / area;
}

// Corner k has coordinate +h on axis i when bit i of k is set, else -h.
// Each face is two triangles wound counterclockwise seen from outside.
void BoxMesh(const Box &box, Mat<Real> &v, Mat<int> &f) {
  if (!(box.halfExtents.array() > 0).all()) {
    Malformed("a Box needs positive half extents");
  }
  v.resize(8, 3);
  for (int k = 0; k < 8; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      v(k, axis) = (k >> axis & 1) ? box.halfExtents(axis)
                                    : -box.halfExtents(axis);
    }
  }
  f.resize(12, 3);
  f << 0, 4, 6, 0, 6, 2, // -x
      1, 3, 7, 1, 7, 5,  // +x
      0, 1, 5, 0, 5, 4,  // -y
      2, 6, 7, 2, 7, 3,  // +y
      0, 2, 3, 0, 3, 1,  // -z
      4, 5, 7, 4, 7, 6;  // +z
}

// Damping coefficients must be non-negative, or they would pump energy in.
void ValidateDamping(const Damping &damping) {
  std::visit(Overloaded{[](const NoDamping &) {},
                        [](const GreenDamping &green) {
                          if (!(green.mu >= 0)) {
                            Malformed("GreenDamping needs a non-negative mu");
                          }
                        },
                        [](const Rayleigh &rayleigh) {
                          if (!(rayleigh.mass >= 0 &&
                                rayleigh.stiffness >= 0)) {
                            Malformed("Rayleigh damping needs non-negative "
                                      "mass and stiffness coefficients");
                          }
                        }},
             static_cast<const DampingModel &>(damping));
}

void ValidateStrandDamping(const Damping &damping) {
  ValidateDamping(damping);
  if (std::holds_alternative<GreenDamping>(damping)) {
    Malformed("a Strand takes NoDamping or Rayleigh damping");
  }
}

auto ContactAt(int vertex, const VertexFaceCollision &hit) -> SoftContact {
  return {.vertex = vertex,
          .triangle = {hit.faceVertices[0], hit.faceVertices[1],
                       hit.faceVertices[2]}};
}

template <System S>
auto StepWith(const Integrator &integrator, const S &system,
              IntegratorState &state, const Vec<Real> &external, Real dt)
    -> StepReport {
  return std::visit(
      [&](const auto &method) {
        return Step(method, system, state, external, dt);
      },
      static_cast<const IntegratorMethod &>(integrator));
}

// One strand: its rod, the system over it and the integrator that steps it.
// The system refers to the rod, so it never moves.
struct StrandSim {
  std::unique_ptr<StrandMesh> mesh;
  DerSystem system;
  Integrator integrator;
  IntegratorState state;
  // Its index in SceneDesc::bodies.
  int index;

  StrandSim(std::unique_ptr<StrandMesh> strandMesh, const Strand &desc,
            std::vector<int> pinned, int bodyIndex)
      : mesh(std::move(strandMesh)),
        system(*mesh->der, pinned, desc.damping, desc.collisionStiffness,
               desc.collisionEpsilon),
        integrator(desc.integrator),
        state{.x = mesh->der->Positions(),
              .v = Vec<Real>::Zero(mesh->der->DOFs())},
        index(bodyIndex) {}
  StrandSim(StrandSim &&) = delete;
};

// One soft body: its mesh, the assembly over it and the integrator that
// steps it. The system refers to the mesh and the material, so it never
// moves.
struct SoftBodySim {
  std::unique_ptr<TetMesh> mesh;
  Material material;
  ElasticSystem system;
  Integrator integrator;
  IntegratorState state;
  // Each vertex's share of the body's volume, which gravity scales.
  std::vector<Real> vertexAreas;
  // Surface vertex indices, and how near a collider triangle one has to be
  // to touch it.
  std::vector<int> surface;
  Real contactRadius;
  // Its index in SceneDesc::bodies.
  int index;

  SoftBodySim(std::unique_ptr<TetMesh> tetMesh, const SoftBody &desc,
              std::vector<int> surfaceVertices, int bodyIndex)
      : mesh(std::move(tetMesh)), material(desc.material),
        system(*mesh, material, desc.damping,
               ContactModel{.stiffness = desc.collisionStiffness,
                            .epsilon = desc.collisionEpsilon,
                            .damping = desc.collisionDamping},
               desc.drag, PlacedFiber(desc.place, desc.fiber)),
        integrator(desc.integrator),
        state{.x = mesh->Positions(), .v = Vec<Real>::Zero(mesh->DOFs())},
        surface(std::move(surfaceVertices)),
        contactRadius(desc.collisionEpsilon), index(bodyIndex) {
    for (int ii = 0; ii < mesh->v.rows(); ++ii) {
      vertexAreas.push_back(mesh->OneRingArea(ii));
    }
  }
  SoftBodySim(SoftBodySim &&) = delete;
};

} // namespace

struct World::Impl {
  SceneDesc desc;
  int steps = 0;
  Vec3<Real> extraAcceleration = Vec3<Real>::Zero();

  std::vector<BodyState> states;
  // Index into the per-kind vectors below, per body in desc.bodies.
  std::vector<int> slot;

  std::vector<std::unique_ptr<StrandSim>> strands;
  std::vector<std::unique_ptr<SoftBodySim>> softBodies;
  std::vector<std::unique_ptr<CollisionMesh>> colliders;
  // Per body, how its integrator did on the last step.
  std::vector<StepReport> reports;

  std::vector<std::vector<VertexFaceCollision>> collisionsPerStrand;
  std::vector<VertexFaceCollision> contacts;

  explicit Impl(SceneDesc sceneDesc) : desc(std::move(sceneDesc)) {
    for (const auto &body : desc.bodies) {
      std::visit([&](const auto &b) { Add(b); }, body);
    }
    for (const auto &action : desc.script) {
      Validate(action);
    }
    collisionsPerStrand.resize(strands.size());
    reports.resize(desc.bodies.size());
  }

  void Add(const Strand &strand) {
    ValidatePlacement(strand.place);
    ValidateStrandDamping(strand.damping);
    auto mesh = std::make_unique<StrandMesh>(
        Placed(strand.place,
               Recentered(strand.points,
                          strand.points.colwise().mean().transpose())));
    std::vector<int> pinned;
    for (const int vertex : strand.pinned) {
      if (vertex < 0 || vertex >= mesh->pinned.rows()) {
        Malformed(std::format("pinned vertex {} is out of range", vertex));
      }
      if (mesh->pinned(vertex) == 0) {
        pinned.push_back(vertex);
      }
      mesh->pinned(vertex) = 1;
    }
    mesh->der->leftAngularVelocity = strand.leftTwistRate;
    mesh->der->rightAngularVelocity = strand.rightTwistRate;
    slot.push_back(static_cast<int>(strands.size()));
    states.emplace_back(mesh.get());
    strands.push_back(std::make_unique<StrandSim>(
        std::move(mesh), strand, std::move(pinned),
        static_cast<int>(states.size()) - 1));
  }

  void Add(const SoftBody &softBody) {
    ValidatePlacement(softBody.place);
    Mat<Real> local;
    auto mesh = std::make_unique<TetMesh>(
        softBody.mesh.path, [&](const Mat<Real> &v, const Mat<int> &t) {
          local = Recentered(v, VolumeCentroid(v, t));
          return Placed(softBody.place, local);
        });
    if (softBody.pinRegion) {
      for (int ii = 0; ii < local.rows(); ++ii) {
        if (StrictlyInside(*softBody.pinRegion, local.row(ii).transpose())) {
          mesh->PinVertex(ii);
        }
      }
    }
    if (!(softBody.collisionStiffness >= 0 && softBody.collisionEpsilon > 0 &&
          softBody.collisionDamping >= 0 && softBody.drag >= 0)) {
      Malformed("a SoftBody needs a positive collisionEpsilon and "
                "non-negative collision stiffness, damping and drag");
    }
    ValidateDamping(softBody.damping);
    std::vector<int> surface(mesh->f.data(), mesh->f.data() + mesh->f.size());
    std::ranges::sort(surface);
    const auto [last, end] = std::ranges::unique(surface);
    surface.erase(last, end);

    slot.push_back(static_cast<int>(softBodies.size()));
    states.emplace_back(mesh.get());
    softBodies.push_back(std::make_unique<SoftBodySim>(
        std::move(mesh), softBody, std::move(surface),
        static_cast<int>(states.size()) - 1));
  }

  void Add(const Collider &collider) {
    ValidatePlacement(collider.place);
    auto mesh = std::make_unique<CollisionMesh>();
    std::visit(Overloaded{
                   [&](const MeshFile &file) {
                     if (!igl::readOBJ(file.path.string(), mesh->v, mesh->f)) {
                       Malformed(std::format("cannot read collider mesh {}",
                                             file.path.string()));
                     }
                   },
                   [&](const Box &box) { BoxMesh(box, mesh->v, mesh->f); },
               },
               collider.shape);
    mesh->rv = Placed(collider.place,
                      Recentered(mesh->v, AreaCentroid(mesh->v, mesh->f)));
    mesh->v = mesh->rv;
    slot.push_back(static_cast<int>(colliders.size()));
    states.emplace_back(mesh.get());
    colliders.push_back(std::move(mesh));
  }

  template <class Kind> auto SlotOf(BodyId id) const -> int {
    if (id.index < 0 || id.index >= std::ssize(desc.bodies) ||
        !std::holds_alternative<Kind>(desc.bodies[id.index])) {
      Malformed(std::format("body {} is not the kind this action targets",
                            id.index));
    }
    return slot[id.index];
  }

  void Validate(const Action &action) const {
    if (action.firstStep < 1 || action.holdSteps < 1 ||
        (action.lastStep && *action.lastStep < action.firstStep)) {
      Malformed(std::format("action window {}..{} hold {} is invalid",
                            action.firstStep, action.lastStep.value_or(-1),
                            action.holdSteps));
    }
    std::visit(Overloaded{
                   [&](const MovePinned &e) {
                     PathSamples(action);
                     const auto &pinned =
                         strands[SlotOf<Strand>(e.strand)]->mesh->pinned;
                     if (e.vertex < 0 || e.vertex >= pinned.rows() ||
                         pinned(e.vertex) != 1) {
                       Malformed(std::format("MovePinned vertex {} is not pinned",
                                             e.vertex));
                     }
                   },
                   [&](const MoveCollider &e) {
                     PathSamples(action);
                     SlotOf<Collider>(e.collider);
                   },
                   [&](const SetTwistRates &e) { SlotOf<Strand>(e.strand); },
                   [&](const SetDamping &e) {
                     SlotOf<Strand>(e.strand);
                     ValidateStrandDamping(e.damping);
                   },
               },
               action.effect);
  }

  void Apply(const Action &action, int step) {
    const int sample = (step - action.firstStep) / action.holdSteps;
    std::visit(
        Overloaded{
            [&](const MovePinned &e) {
              const Real dx = Vec<Real>::LinSpaced(PathSamples(action), e.fromDx,
                                                   e.toDx)(sample);
              // A kinematic target: the vertex is pinned, so the integrator
              // holds it where this puts it.
              StrandSim &strand = *strands[slot[e.strand.index]];
              DiscreteElasticRod &rod = *strand.mesh->der;
              rod.vertices(e.vertex, 0) = rod.restVertices(e.vertex, 0) + dx;
              strand.state.x(3 * e.vertex) = rod.vertices(e.vertex, 0);
            },
            [&](const MoveCollider &e) {
              const int n = PathSamples(action);
              Vec3<Real> offset;
              for (int axis = 0; axis < 3; ++axis) {
                offset(axis) =
                    Vec<Real>::LinSpaced(n, e.from(axis), e.to(axis))(sample);
              }
              colliders[slot[e.collider.index]]->Translate(offset);
            },
            [&](const SetTwistRates &e) {
              auto &der = *strands[slot[e.strand.index]]->mesh->der;
              der.leftAngularVelocity = e.left;
              der.rightAngularVelocity = e.right;
            },
            [&](const SetDamping &e) {
              strands[slot[e.strand.index]]->system.SetDamping(e.damping);
            },
        },
        action.effect);
  }

  // The body's surface vertices within its contact radius of a collider
  // triangle, one contact per vertex and triangle.
  auto SoftContacts(const SoftBodySim &body) const
      -> std::vector<SoftContact> {
    const TetMesh &mesh = *body.mesh;
    const std::vector<int> &surface = body.surface;
    Mat<Real> positions(std::ssize(surface), 3);
    for (int row = 0; row < std::ssize(surface); ++row) {
      positions.row(row) = mesh.v.row(surface[row]);
    }
    std::vector<SoftContact> found;
    for (const auto &collider : colliders) {
      for (const auto &hit :
           collider->DetectFaceCollisions(body.contactRadius, positions)) {
        found.push_back(ContactAt(surface[hit.vertexIndex], hit));
      }
    }
    return found;
  }

  void Step() {
    const int step = steps + 1;
    for (const auto &action : desc.script) {
      if (step >= action.firstStep &&
          (!action.lastStep || step <= *action.lastStep)) {
        Apply(action, step);
      }
    }

    const Vec3<Real> gravity = desc.gravity + extraAcceleration;
    // One strand per index, and each writes only its own slots, so the
    // result does not depend on the thread count.
    igl::parallel_for(strands.size(), [&](int ii) {
      StrandSim &strand = *strands[ii];
      auto &found = collisionsPerStrand[ii];
      found.clear();
      for (const auto &collider : colliders) {
        auto hits = collider->DetectFaceCollisions(desc.collisionEnvelope,
                                                   strand.mesh->der->vertices);
        found.insert(found.end(), std::make_move_iterator(hits.begin()),
                     std::make_move_iterator(hits.end()));
      }
      std::vector<SoftContact> strandContacts;
      strandContacts.reserve(found.size());
      for (const auto &hit : found) {
        strandContacts.push_back(ContactAt(hit.vertexIndex, hit));
      }
      strand.system.SetContacts(std::move(strandContacts));
      // Gravity is a force per strand vertex, not an acceleration.
      Vec<Real> external(strand.system.Dofs());
      for (int vertex = 0; vertex < external.size() / 3; ++vertex) {
        external.segment<3>(3 * vertex) = gravity;
      }
      reports[strand.index] = StepWith(strand.integrator, strand.system,
                                       strand.state, external, desc.dt);
      strand.mesh->der->Advance(strand.state.x, desc.dt);
    });
    contacts.clear();
    for (const auto &found : collisionsPerStrand) {
      contacts.insert(contacts.end(), found.begin(), found.end());
    }

    // Contacts are found once, at the start of the step, and held fixed
    // through the integrator's solve.
    for (const auto &body : softBodies) {
      body->system.SetContacts(SoftContacts(*body));
      Vec<Real> external(body->mesh->DOFs());
      for (int ii = 0; ii < std::ssize(body->vertexAreas); ++ii) {
        external.segment<3>(3 * ii) = body->vertexAreas[ii] * gravity;
      }
      reports[body->index] = StepWith(body->integrator, body->system,
                                      body->state, external, desc.dt);
      body->mesh->SetPositions(body->state.x);
    }
    steps = step;
  }
};

World::World(SceneDesc desc) : impl(std::make_unique<Impl>(std::move(desc))) {}
World::~World() = default;
World::World(World &&) noexcept = default;
auto World::operator=(World &&) noexcept -> World & = default;

void World::Step() {
  if (!Finished()) {
    impl->Step();
  }
}

void World::SetExtraAcceleration(const Vec3<Real> &acceleration) {
  impl->extraAcceleration = acceleration;
}

auto World::Steps() const -> int { return impl->steps; }

auto World::Finished() const -> bool {
  return impl->desc.lastStep && impl->steps >= *impl->desc.lastStep;
}

auto World::Desc() const -> const SceneDesc & { return impl->desc; }

auto World::Bodies() const -> std::span<const BodyState> {
  return impl->states;
}

auto World::StepReports() const -> std::span<const StepReport> {
  return impl->reports;
}

auto World::Contacts() const -> std::span<const VertexFaceCollision> {
  return impl->contacts;
}

} // namespace sim
