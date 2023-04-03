#include "TetMesh.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/boundary_facets.h>
#include <string>

void TetMesh::Tetrahedralize() {
  static const std::string switches = "zpQ";

  igl::copyleft::tetgen::tetrahedralize(Mat<Real>(v), Mat<int>(f),
                                        switches, v, t, f);
  igl::boundary_facets(t, f);
  f.rowwise().reverseInPlace();
}
