#pragma once

#include <LibMath.h>

#include <vector>

namespace sim {

// Constraints as a projection (Baraff & Witkin 1998, "Large Steps in Cloth
// Simulation", section 5): S = I - sum n n^T over every constrained direction.
// A pinned vertex constrains all three of its DOFs, so S zeroes them.
struct Constraints {
  int dofs = 0;
  // Vertex indices; vertex i owns DOFs 3i, 3i + 1 and 3i + 2.
  std::vector<int> pinned;

  [[nodiscard]] auto Filter() const -> SparseMat<Real>;
};

} // namespace sim
