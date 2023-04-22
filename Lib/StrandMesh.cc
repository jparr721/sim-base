#include "OpenGL.h"
#include <StrandMesh.h>

StrandMesh::StrandMesh(const Mat<Real> &points) {
  // Fill modified bishop frames from the bag of points
  ASSERT(points.rows() % 2 == 0, "The number of points must be even");

  // Create the rod segments
  WalkBishopFrames(points);

#ifndef NDEBUG
  std::cout << "Created " << rodSegments.size() << " rod segments" << std::endl;
#endif

  // Create bending points
  for (int ii = 0; ii < points.rows() - 2; ++ii) {
    const Vec3<Real> p0 = points.row(ii);
    const Vec3<Real> p1 = points.row(ii + 1);
    const Vec3<Real> p2 = points.row(ii + 2);

    // We need two edges for the curvature binormal
    // e^i-1
    const Vec3<Real> e0 = p1 - p0;

    // e^i
    const Vec3<Real> e1 = p2 - p1;

    // (kb)_i  = 2 * e^i-1 x e^i / (||e^i-1|| ||e^i|| + e^i-1 . e^i)
    // Equation from Bergou et al. '08, eq 1.
    const Vec3<Real> curvatureBinormal =
        2 * e0.cross(e1) / (e0.norm() * e1.norm() + e0.dot(e1));

    // Insert the bend segment and we are done.
    bendableSegments.emplace_back(p0, p1, p2, curvatureBinormal);
  }

#ifndef NDEBUG
  std::cout << "Created " << bendableSegments.size() << " bendable segments"
            << std::endl;
#endif

  UpdateMaterialCurvature();
}

void StrandMesh::Draw() {
  glBegin(GL_LINES);
  glLineWidth(10);
  for (const auto &rodSegment : rodSegments) {
    // Draw the bishop frame from t0, u, v
    glColor3f(1, 0, 0);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv((rodSegment.x0 + rodSegment.bishopFrame.t).eval().data());

    glColor3f(0, 1, 0);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv((rodSegment.x0 + rodSegment.bishopFrame.u).eval().data());

    glColor3f(0, 0, 1);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv((rodSegment.x0 + rodSegment.bishopFrame.v).eval().data());
  }
  glEnd();

  glBegin(GL_POINTS);
  glPointSize(20);
  for (const auto &rodSegment : rodSegments) {
    glColor3f(1, 1, 1);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv(rodSegment.x1.data());
  }
  glEnd();
}

void StrandMesh::WalkBishopFrames(const Mat<Real> &points) {
  rodSegments.clear();

  // Before we can parallel transport, we must first initialize the frame at
  // point 0.

  // The tangent is the edge from x0 to x1 (always defined).
  Vec3<Real> tangent = (points.row(1) - points.row(0)).normalized();

  // The first bishop frame vector is a vector orthogonal to the tangent
  Vec3<Real> director = GetOrthogonalVector(tangent);

  // Now parallel transport the director with the tangent
  // CHECK: I think this just returns "director" into u...
  Vec3<Real> u = ParallelTransport(director, tangent, tangent);

  // Remove any similarity between the tangent and u
  u = (u - u.dot(tangent) * tangent).normalized();

  director = u;

  // The next director is orthogonal to u, since u is already orthogonal to
  // the first director
  Vec3<Real> director2 = tangent.cross(u);

  // Create our first frame
  rodSegments.emplace_back(points.row(0), points.row(1), tangent, director,
                           director2);

  // Parallel transport the rest from the starting frame
  for (int ii = 1; ii < points.rows() - 1; ++ii) {
    const Vec3<Real> p0 = points.row(ii);
    const Vec3<Real> p1 = points.row(ii + 1);

    Vec3<Real> bitangent = (p1 - p0).normalized();

    u = ParallelTransport(u, tangent, bitangent);

    director = u;
    director2 = bitangent.cross(u);
    tangent = bitangent;

    rodSegments.emplace_back(p0, p1, tangent, director, director2);
  }
}


void UpdateQuasistaticFrame() {

}

void StrandMesh::UpdateBishopFrames() {
  Mat<Real> points;
  points.resize(rodSegments.size() + 1, 3);
  for (int ii = 0; ii < rodSegments.size() + 1; ++ii) {
    points.row(ii) = rodSegments.at(ii).x0;
  }
  points.row(rodSegments.size()) = rodSegments.back().x1;

  WalkBishopFrames(points);
}

void StrandMesh::UpdateMaterialCurvature() {
  for (int ii = 0; ii < rodSegments.size(); ++ii) {
    for (int jj = ii - 1; jj <= ii; ++jj) {
      if (jj < 0 || jj >= rodSegments.size() - 1) {
        continue;
      }

      const auto &rodSegment = rodSegments.at(jj);

      // [Bergou et al. '08, 4.1.2]
      const Vec3<Real> m1 =
          std::cos(rodSegment.theta) * rodSegment.bishopFrame.u +
          std::sin(rodSegment.theta) * rodSegment.bishopFrame.v;
      const Vec3<Real> m2 =
          -std::sin(rodSegment.theta) * rodSegment.bishopFrame.u +
          std::cos(rodSegment.theta) * rodSegment.bishopFrame.v;

      // [Bergou et al. '08, Equation 2]
      auto &bendSegment = bendableSegments.at(jj);
      if (ii == jj) {
        bendSegment.gamma = Vec2<Real>(bendSegment.curvatureBinormal.dot(m2),
                                       -bendSegment.curvatureBinormal.dot(m1));
      } else {
        bendSegment.prevGamma =
            Vec2<Real>(bendSegment.curvatureBinormal.dot(m2),
                       -bendSegment.curvatureBinormal.dot(m1));
      }
    }
  }
}

ModifiedBishopFrame::ModifiedBishopFrame(const Vec3<Real> &x0,
                                         const Vec3<Real> &x1,
                                         const Vec3<Real> &t,
                                         const Vec3<Real> &u,
                                         const Vec3<Real> &v)
    : x0(x0), x1(x1), edge((x0 - x1).eval()), bishopFrame({t, u, v}), theta(0) {
}

auto operator<<(std::ostream &os, const ModifiedBishopFrame &frame)
    -> std::ostream & {
  os << "x0 = " << frame.x0.transpose() << ", ";
  os << "x1 = " << frame.x1.transpose() << ", ";
  os << "e0 = " << frame.edge.transpose() << ", ";
  os << "t0 = " << frame.bishopFrame.t.transpose() << ", ";
  os << "u = " << frame.bishopFrame.u.transpose() << ", ";
  os << "v = " << frame.bishopFrame.v.transpose() << ", ";
  os << "theta = " << frame.theta << ", ";
  return os;
}
