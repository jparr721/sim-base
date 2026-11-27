#pragma once

#include <LibMath.h>

#include <array>

namespace sim {

// A body vertex near a static triangle. The triangle never moves in response,
// so only the vertex's DOFs see the contact.
struct SoftContact {
  int vertex;
  std::array<Vec3<Real>, 3> triangle;
};

// Frictionless penalty contact against static triangles. Stiffness and
// damping are per unit vertex mass, so every vertex has the same contact
// frequency whatever the size of its tets.
struct ContactModel {
  // 1/s^2. Vertex i stores m_i stiffness (d - eps)^2, with d its distance to
  // the triangle (VertexFaceSqrtCollision).
  Real stiffness = 0;
  Real epsilon = 0;
  // 1/s. Vertex i feels -m_i damping (n . v_i) n, with n the triangle normal.
  Real damping = 0;
};

} // namespace sim
