#include "TetMesh.h"
#include <igl/boundary_facets.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <string>

void TetMesh::Tetrahedralize() {
  static const std::string switches = "zpQ";

  igl::copyleft::tetgen::tetrahedralize(Mat<Real>(v), Mat<int>(f), switches, v,
                                        t, f);
  igl::boundary_facets(t, f);

  // The faces come out of this function in the wrong winding order. So
  // this fixes that.
  f.rowwise().reverseInPlace();
}
