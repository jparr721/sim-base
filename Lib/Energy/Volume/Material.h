#pragma once

#include <Energy/Volume/Arap.h>
#include <Energy/Volume/LinearElasticity.h>
#include <Energy/Volume/NeoHookeanBonetWood.h>
#include <Energy/Volume/StVenantKirchhoff.h>
#include <Energy/Volume/StableNeoHookean.h>
#include <Energy/Volume/StableNeoHookeanBarrier.h>
#include <Energy/Volume/SymmetricDirichlet.h>

#include <variant>

namespace sim {

// Each alternative is built from Lame parameters, e.g.
// StableNeoHookean{.lame = Lame::FromYoungs(5.0, 0.45)}, and resists
// deformation in every direction. The fiber energies, which do not, are a
// separate FiberMaterial that adds to one of these.
using Material = std::variant<StableNeoHookean,
                              StVenantKirchhoff,
                              LinearElasticity,
                              NeoHookeanBonetWood,
                              Arap,
                              SymmetricDirichlet,
                              StableNeoHookeanBarrier>;

} // namespace sim
