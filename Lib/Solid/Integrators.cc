#include <Solid/Integrators.h>

#include <Eigen/IterativeLinearSolvers>

#include <vector>

namespace sim::detail {

// S A S + I - S keeps A's free rows and columns and puts 1 on each
// constrained diagonal (Baraff & Witkin 1998, section 5.3).
auto SolveFiltered(const SparseMat<Real> &a, const Constraints &constraints,
                   const Vec<Real> &b) -> CgSolve {
  std::vector<bool> free(constraints.dofs, true);
  for (const int vertex : constraints.pinned) {
    for (int axis = 0; axis < 3; ++axis) {
      free[3 * vertex + axis] = false;
    }
  }
  SparseMat<Real> filtered = a;
  filtered.prune([&](Eigen::Index row, Eigen::Index col, Real) {
    return (free[row] && free[col]) || row == col;
  });
  for (const int vertex : constraints.pinned) {
    for (int axis = 0; axis < 3; ++axis) {
      filtered.coeffRef(3 * vertex + axis, 3 * vertex + axis) = 1;
    }
  }
  Eigen::ConjugateGradient<SparseMat<Real>, Eigen::Lower | Eigen::Upper> cg;
  cg.setTolerance(1e-8);
  cg.compute(filtered);
  return {.dx = cg.solve(b), .iterations = static_cast<int>(cg.iterations())};
}

void Project(const Constraints &constraints, Vec<Real> &y) {
  for (const int vertex : constraints.pinned) {
    y.segment<3>(3 * vertex).setZero();
  }
}

auto LumpedInverse(const SparseMat<Real> &mass) -> Vec<Real> {
  return Vec<Real>(mass.diagonal()).cwiseInverse();
}

} // namespace sim::detail
