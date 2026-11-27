// A game's own math header, as the isometric voxel game embedding sim has.
// The core's headers must compile after it, so they may not declare these
// names globally or define these macros.
constexpr double PI = 3.0;
constexpr double E = 2.0;
#define ASSERT(x)
#define VERSION 1
using Real = float;

#include <Sim/World.h>
#include <Sim/Scene.h>

static_assert(sizeof(Real) == sizeof(float));
static_assert(sizeof(sim::Real) == sizeof(double));
static_assert(PI == 3.0 && E == 2.0 && VERSION == 1);
