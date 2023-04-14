#pragma once

#include <LibMath.h>
#include <fstream>
#include <igl/boundary_facets.h>

template <typename T>
auto ReadTobjFile(const std::string &filename, Mat<T> &verts, Mat<int> &tets,
                  Mat<int> &faces) -> bool {
  std::vector<Vec3<T>> vertsVec;
  std::vector<Vec4<int>> tetsVec;

  FILE *file = fopen(filename.c_str(), "r");

  if (file == nullptr) {
    return false;
  }

  char nextChar = getc(file);

  // get the vertices
  while (nextChar == 'v') {
    ungetc(nextChar, file);

    double v[3];
    fscanf(file, "v %lf %lf %lf\n", &v[0], &v[1], &v[2]);
    vertsVec.push_back(Vec3<T>(v[0], v[1], v[2]));

    nextChar = getc(file);
  }
  if (nextChar == EOF) {
    return false;
  }

  // get the tets
  while (nextChar == 't') {
    ungetc(nextChar, file);

    Vec4<int> tet;
    fscanf(file, "t %i %i %i %i\n", &tet[0], &tet[1], &tet[2], &tet[3]);
    tetsVec.push_back(tet);
    nextChar = getc(file);
  }
  fclose(file);

  // convert to eigen matrices
  verts.resize(vertsVec.size(), 3);
  for (int i = 0; i < vertsVec.size(); i++) {
    verts.row(i) = vertsVec[i];
  }

  tets.resize(tetsVec.size(), 4);
  for (int i = 0; i < tetsVec.size(); i++) {
    tets.row(i) = tetsVec[i];
  }

  igl::boundary_facets(tets, faces);

  // Faces are out of order, so we fix the winding
  faces.rowwise().reverse();
  return true;
}
