#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <LibMath.h>

class StrandMesh {
public:
  std::shared_ptr<DiscreteElasticRod> der;

  StrandMesh(const Mat<Real> &points);
  void Draw();
};