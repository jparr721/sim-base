#pragma once

#include <Energy/Volume/AnisotropicArap.h>
#include <Energy/Volume/AnisotropicFung.h>
#include <Energy/Volume/AnisotropicStVenantKirchhoff.h>

#include <variant>

namespace sim {

// A fiber energy resists stretch only along its fiber, so a body made of one
// alone collapses. It is never a body's whole Material: a SoftBody adds it to
// its isotropic material. Each alternative takes Lame parameters (only mu is
// used) and a uniform unit fiber direction.
using FiberMaterial = std::variant<AnisotropicStVenantKirchhoff,
                                   AnisotropicArap,
                                   AnisotropicFung>;

} // namespace sim
