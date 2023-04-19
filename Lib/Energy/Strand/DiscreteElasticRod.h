#pragma once

#include <LibMath.h>

class DiscreteElasticRod {
public:
  DiscreteElasticRod(Real beta, Real alpha) : beta(beta), alpha(alpha){}

  void Energy();
  void Gradient();
  void Hessian();

private:
  // The twisting energy stiffness
  Real beta;

  // The bending energy stiffness
  Real alpha;
};