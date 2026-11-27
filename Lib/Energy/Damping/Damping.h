#pragma once

#include <Energy/Damping/GreenDamping.h>

#include <variant>

namespace sim {

// A body with no material damping. Contact damping and drag still apply.
struct NoDamping {};

// Rayleigh damping: the force -(mass M + stiffness K(x)) v, with M the mass
// matrix and K(x) the clamped elastic stiffness at the current positions, so
// the damping matrix is PSD. mass (1/s) damps every motion, rigid ones too,
// like SoftBody::drag; stiffness (s) damps each elastic mode in proportion to
// its stiffness, so high frequencies die fastest.
struct Rayleigh {
  Real mass = 0;
  Real stiffness = 0;
};

using DampingModel = std::variant<NoDamping, GreenDamping, Rayleigh>;

// No default constructor, so a body that leaves out its damping does not
// compile.
struct Damping : DampingModel {
  using DampingModel::DampingModel;
  Damping() = delete;
};

} // namespace sim
