#include "World.h"

#include <Integrator/ForwardEulerStrand.h>
#include <Integrator/ForwardEulerVolume.h>
#include <igl/parallel_for.h>
#include <igl/readOBJ.h>

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

} // namespace

struct World::Impl {
  SceneDesc desc;
  int steps = 0;
  Vec3<Real> extraAcceleration = Vec3<Real>::Zero();

  std::vector<BodyState> states;
  // Index into the per-kind vectors below, per body in desc.bodies.
  std::vector<int> slot;

  std::vector<std::shared_ptr<StrandMesh>> strands;
  std::vector<std::unique_ptr<ForwardEulerStrand>> strandIntegrators;
  std::vector<std::shared_ptr<TetMesh>> softBodies;
  std::vector<std::unique_ptr<ForwardEulerVolume>> volumeIntegrators;
  std::vector<std::unique_ptr<CollisionMesh>> colliders;

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
  }

  void Add(const Strand &strand) {
    ValidatePlacement(strand.place);
    auto mesh = std::make_shared<StrandMesh>(
        Placed(strand.place,
               Recentered(strand.points,
                          strand.points.colwise().mean().transpose())));
    for (const int vertex : strand.pinned) {
      if (vertex < 0 || vertex >= mesh->pinned.rows()) {
        Malformed(std::format("pinned vertex {} is out of range", vertex));
      }
      mesh->pinned(vertex) = 1;
    }
    mesh->der->leftAngularVelocity = strand.leftTwistRate;
    mesh->der->rightAngularVelocity = strand.rightTwistRate;
    slot.push_back(static_cast<int>(strands.size()));
    states.emplace_back(mesh.get());
    strandIntegrators.push_back(std::make_unique<ForwardEulerStrand>(
        mesh, nullptr, desc.dt, strand.collisionStiffness,
        strand.collisionEpsilon));
    strands.push_back(std::move(mesh));
  }

  void Add(const SoftBody &softBody) {
    ValidatePlacement(softBody.place);
    Mat<Real> local;
    auto mesh = std::make_shared<TetMesh>(
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
    slot.push_back(static_cast<int>(softBodies.size()));
    states.emplace_back(mesh.get());
    volumeIntegrators.push_back(
        std::make_unique<ForwardEulerVolume>(mesh, softBody.material, desc.dt));
    softBodies.push_back(std::move(mesh));
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
                     const auto &pinned = strands[SlotOf<Strand>(e.strand)]->pinned;
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
                   [&](const SetDamping &e) { SlotOf<Strand>(e.strand); },
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
              strands[slot[e.strand.index]]->TranslatePinnedX(dx, e.vertex);
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
              auto &der = *strands[slot[e.strand.index]]->der;
              der.leftAngularVelocity = e.left;
              der.rightAngularVelocity = e.right;
            },
            [&](const SetDamping &e) {
              strandIntegrators[slot[e.strand.index]]->damping = e.coefficient;
            },
        },
        action.effect);
  }

  void Step() {
    const int step = steps + 1;
    for (const auto &action : desc.script) {
      if (step >= action.firstStep &&
          (!action.lastStep || step <= *action.lastStep)) {
        Apply(action, step);
      }
    }

    igl::parallel_for(strands.size(), [&](int ii) {
      auto &found = collisionsPerStrand[ii];
      found.clear();
      for (const auto &collider : colliders) {
        auto hits = collider->DetectFaceCollisions(desc.collisionEnvelope,
                                                   strands[ii]->der->vertices);
        found.insert(found.end(), std::make_move_iterator(hits.begin()),
                     std::make_move_iterator(hits.end()));
      }
    });
    contacts.clear();
    for (const auto &found : collisionsPerStrand) {
      contacts.insert(contacts.end(), found.begin(), found.end());
    }

    const Vec3<Real> gravity = desc.gravity + extraAcceleration;
    igl::parallel_for(strandIntegrators.size(), [&](int ii) {
      auto &integrator = strandIntegrators[ii];
      integrator->vertexFaceCollisions = collisionsPerStrand[ii];
      integrator->AddGravity(gravity);
      integrator->Step();
    });
    for (auto &integrator : volumeIntegrators) {
      integrator->AddGravity(gravity);
      integrator->Step();
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

auto World::Contacts() const -> std::span<const VertexFaceCollision> {
  return impl->contacts;
}

} // namespace sim
