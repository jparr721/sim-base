#pragma once

#include "AdaptedFrame.h"
#include <Energy/ElementEnergy.h>
#include <Energy/Spring/MassSpring.h>
#include <LibMath.h>

#include <Eigen/Eigenvalues>

namespace sim {

constexpr Real gBendingModulus = 0.1;
constexpr Real gTwistingModulus = 1;

// h with its negative eigenvalues set to 0.
template <int N> auto ProjectToPsd(const MatN<N> &h) -> MatN<N> {
  const Eigen::SelfAdjointEigenSolver<MatN<N>> eigen(h);
  return eigen.eigenvectors() * eigen.eigenvalues().cwiseMax(0).asDiagonal() *
         eigen.eigenvectors().transpose();
}

// Gradient of a curvature binormal kb_i with respect to the three vertices it
// depends on: x_i, x_{i+1} and x_{i+2}.
struct KbGradient {
  Mat3<Real> prev;
  Mat3<Real> mid;
  Mat3<Real> next;
};

// Bending at one interior vertex over x = (x_{i-1}, x_i, x_{i+1}) (Bergou et
// al. 2008, isotropic rod): Psi = modulus |kb|^2 / (lbar0 + lbar1), with
// kb = 2 e0 x e1 / (lbar0 lbar1 + e0 . e1) and lbar the two edges' rest
// lengths.
struct Bending {
  Real modulus;
  Real restLength0;
  Real restLength1;

  [[nodiscard]] auto Psi(const VecN<9> &x) const -> Real;
  [[nodiscard]] auto Gradient(const VecN<9> &x) const -> VecN<9>;
  [[nodiscard]] auto Hessian(const VecN<9> &x) const -> MatN<9>;
  [[nodiscard]] auto ClampedHessian(const VecN<9> &x) const -> MatN<9>;
};

static_assert(ElementEnergy<Bending, 9>);

// The uniform-twist force at one interior vertex over x = (x_{i-1}, x_i,
// x_{i+1}): scale times the holonomy gradient of psi_i, with
// grad_{i-1} psi_i = kb / (2 lbar0) and grad_{i+1} psi_i = -kb / (2 lbar1)
// (Bergou et al. 2008, section 7). DiscreteElasticRod::Forces sums it.
// scale = 2 beta (theta^n - theta^0) / L is held over a step, so the force
// is not the gradient of an energy in x, and its Jacobian is not symmetric.
struct TwistForce {
  Real scale;
  Real restLength0;
  Real restLength1;

  // -dForce/dx.
  [[nodiscard]] auto Stiffness(const VecN<9> &x) const -> MatN<9>;
  // The symmetric part of Stiffness with its negative eigenvalues set to 0.
  [[nodiscard]] auto ClampedStiffness(const VecN<9> &x) const -> MatN<9>;
};

// A strand's rest shape, its twist, and its frames at the current positions.
// The DOFs are the vertex positions. Theta is not one: the rod is isotropic,
// so bending reads |kb| alone, and the interior thetas of a uniform twist
// are eliminated in closed form (Bergou et al. 2008, section 7), leaving the
// two end angles, which the twist rates drive.
class DiscreteElasticRod {
public:
  int nRods;
  Real totalRestLength;
  Real rightAngularVelocity = 0.0;
  Real leftAngularVelocity = 0.0;

  // Every edge is a MassSpring with this stiffness and its own rest length.
  Real springStiffness = 100000;
  Real vertexMass = 1.0 / 5;

  Mat<Real> vertices;
  Mat<Real> restVertices;

  std::vector<Edge> edges;
  std::vector<Edge> restEdges;
  std::vector<BishopFrame> frames;

  std::vector<Vec3<Real>> kbs;

  Vec<Real> thetas;

  explicit DiscreteElasticRod(Mat<Real> vertices);

  [[nodiscard]] SIM_INLINE auto DOFs() const -> int { return 3 * vertices.rows(); }
  SIM_INLINE auto Positions() const -> Vec<Real> {
    return RowwiseFlatten(vertices);
  }

  // Moves the centerline to x, turns the end material frames by their twist
  // rates over dt, and carries the frames along.
  void Advance(const Vec<Real> &x, Real dt);

  // Bending, the end twist and the springs at positions x, with the thetas
  // held.
  [[nodiscard]] auto Forces(const Vec<Real> &x) const -> Vec<Real>;
  // Bending and spring energy at x, plus the twist energy
  // beta (theta^n - theta^0)^2 / L, which the held thetas make constant in x.
  [[nodiscard]] auto Energy(const Vec<Real> &x) const -> Real;

  [[nodiscard]] auto BendingAt(int vertex) const -> Bending;
  [[nodiscard]] auto TwistAt(int vertex) const -> TwistForce;
  [[nodiscard]] auto SpringAt(int edge) const -> MassSpring;
  // The 9 positions bending and twist at interior vertex `vertex` read.
  [[nodiscard]] static auto Stencil(const Vec<Real> &x, int vertex) -> VecN<9>;

private:
  void Initialize();
  void UpdateBishopFrames();
};

} // namespace sim
