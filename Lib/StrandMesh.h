#pragma once

#include <LibMath.h>

class StrandMesh {
  // Simulated vertices
  Mat<Real> v;

  // Rest vertices
  Mat<Real> rv;

  // Simulated edges
  Mat<int> e;

  // Rest edges
  Mat<int> re;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<int> pinned;

  // Per-element deformation gradients
  std::vector<Mat<Real>> fs;
};
