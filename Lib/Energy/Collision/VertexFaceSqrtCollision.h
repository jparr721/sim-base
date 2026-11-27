#pragma once

#include <Energy/ElementEnergy.h>

namespace sim {

// Vertex-face penalty contact (Dynamic Deformables, Kim & Eberle 2022, the
// collision chapter's vertex-face energy). x stacks the colliding vertex, then
// the triangle's three vertices. Psi = k (phi - eps)^2 pushes the vertex to
// eps in front of the face, where phi is a signed distance to the triangle:
//  - Over the face, the distance s to its plane, positive in front.
//  - Beside it, in front, the distance d to its closest edge point.
//  - Beside it, behind, d - 2 s^2 / d. That meets d on the plane and s at the
//    face's edge, so a vertex can slide off the face and cross the plane
//    while its contact is held fixed without Psi or its gradient jumping.
// The front is the side (x1 - x2) x (x3 - x2) points to, with x1..x3 the
// triangle's vertices in order, so it is the back of a counterclockwise
// triangle.
//
// d is a minimum over the triangle's barycentric coordinates, so its gradient
// holds them fixed (the envelope theorem), and its Hessian adds how the closest
// point slides as x moves. Psi and its gradient are continuous everywhere but
// on the triangle's boundary, where d has a cone. The Hessian also jumps on the
// plane beside the face and between the edge and corner regions.
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
