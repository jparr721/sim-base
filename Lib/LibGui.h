#pragma once

#include <LibMath.h>
#include <OpenGL.h>

void DrawSurfaceMesh(const Mat<Real> &v, const Mat<int> &f);
void DrawCylinder(const Vec3<Real> &p0, const Vec3<Real> &p1, Real radius);