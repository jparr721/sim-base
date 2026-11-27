#include "World.h"

#include <Energy/Volume/SNH.h>
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
    auto mesh = std::make_shared<StrandMesh>(strand.points);
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
    auto mesh = std::make_shared<TetMesh>(softBody.mesh);
    for (int ii = 0; ii < mesh->v.rows(); ++ii) {
      if (StrictlyInside(softBody.pinRegion, mesh->v.row(ii).transpose())) {
        mesh->PinVertex(ii);
      }
    }
    const auto material = std::visit(
        [](const NeoHookean &m) -> std::shared_ptr<HyperelasticMaterial> {
          return std::make_shared<SNH>(m.youngsModulus, m.poissonRatio);
        },
        softBody.material);
    slot.push_back(static_cast<int>(softBodies.size()));
    states.emplace_back(mesh.get());
    volumeIntegrators.push_back(
        std::make_unique<ForwardEulerVolume>(mesh, material, desc.dt));
    softBodies.push_back(std::move(mesh));
  }

  void Add(const Collider &collider) {
    auto mesh = std::make_unique<CollisionMesh>();
    if (!igl::readOBJ(collider.mesh.string(), mesh->v, mesh->f)) {
      Malformed(std::format("cannot read collider mesh {}",
                            collider.mesh.string()));
    }
    mesh->rv = mesh->v;
    mesh->Translate(collider.offset);
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
