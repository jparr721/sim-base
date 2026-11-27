#pragma once

#include <Energy/Volume/LinearElasticity.h>
#include <Energy/Volume/NeoHookeanBonetWood.h>
#include <Energy/Volume/StVenantKirchhoff.h>
#include <Energy/Volume/StableNeoHookean.h>

#include <variant>

namespace sim {

// Each alternative is built from Lame parameters, e.g.
// StableNeoHookean{.lame = Lame::FromYoungs(5.0, 0.45)}.
using Material =
    std::variant<StableNeoHookean,
                 StVenantKirchhoff,
                 LinearElasticity,
                 NeoHookeanBonetWood>;

} // namespace sim
