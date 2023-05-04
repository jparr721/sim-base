#pragma once

#include <LibMath.h>
#include <OpenGL.h>

void DrawSurfaceMesh(const Mat<Real> &v, const Mat<int> &f);
void DrawCylinder(const Vec3<Real> &p0, const Vec3<Real> &p1, Real radius);
void DrawText(const std::string &text, int windowWidth, int windowHeight);
void DrawGLGrid(int size, float spacing);
void DrawCenterAxes();
