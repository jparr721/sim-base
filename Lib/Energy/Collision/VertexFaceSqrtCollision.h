#pragma once

#include <Energy/ElementEnergy.h>

namespace sim {

// Vertex-face penalty contact (Dynamic Deformables, Kim & Eberle 2022, the
// collision chapter's vertex-face energy). x stacks the colliding vertex, then
// the triangle's three vertices. With d the distance from the vertex to its
// closest point on the triangle and sigma = -1 when the vertex is behind the
// face, Psi = k (d - sigma eps)^2: it pushes the vertex to eps in front of the
// face. The front is the side (x1 - x2) x (x3 - x2) points to, with x1..x3 the
// triangle's vertices in order, so it is the back of a counterclockwise
// triangle.
//
// d is a minimum over the triangle's barycentric coordinates, so the gradient
// holds them fixed (the envelope theorem). The Hessian adds how the closest
// point slides as x moves. Psi is smooth away from the triangle's plane and
// from the boundaries between its face, edge and corner regions.
class VertexFaceSqrtCollision {
public:
  Real collisionStiffness;
  Real collisionEpsilon;
  VertexFaceSqrtCollision(Real collisionStiffness, Real collisionEpsilon);

  [[nodiscard]] auto Psi(const VecN<12> &x) const -> Real;
  [[nodiscard]] auto Gradient(const VecN<12> &x) const -> VecN<12>;
  [[nodiscard]] auto Hessian(const VecN<12> &x) const -> MatN<12>;
  // Hessian with its negative eigenvalues set to 0.
  [[nodiscard]] auto ClampedHessian(const VecN<12> &x) const -> MatN<12>;
};

static_assert(ElementEnergy<VertexFaceSqrtCollision, 12>);

} // namespace sim
