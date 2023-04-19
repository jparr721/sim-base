#include "OpenGL.h"
#include <LibGui.h>
#include <StrandMesh.h>

StrandMesh::StrandMesh(const Mat<Real> &points) {
  // Fill modified bishop frames from the bag of points
  ASSERT(points.rows() % 2 == 0, "The number of points must be even");

  WalkBishopFrames(points);

#ifndef NDEBUG
  std::cout << "Created " << rodSegments.size() << " rod segments" << std::endl;
#endif
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
  glPointSize(10);
  for (const auto &rodSegment : rodSegments) {
    glColor3f(1, 1, 1);
    glVertex3dv(rodSegment.x0.data());
    glVertex3dv(rodSegment.x1.data());
  }
  glEnd();
}

void StrandMesh::WalkBishopFrames(const Mat<Real> &points) {
  // Remove existing segments
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

ModifiedBishopFrame::ModifiedBishopFrame(const Vec3<Real> &x0,
                                         const Vec3<Real> &x1,
                                         const Vec3<Real> &t,
                                         const Vec3<Real> &u,
                                         const Vec3<Real> &v)
    : x0(x0), x1(x1), edge((x0 - x1).eval()), bishopFrame({t, u, v}), theta0(0),
      theta1(0) {}

auto operator<<(std::ostream &os, const ModifiedBishopFrame &frame)
    -> std::ostream & {
  os << "x0 = " << frame.x0.transpose() << ", ";
  os << "x1 = " << frame.x1.transpose() << ", ";
  os << "e0 = " << frame.edge.transpose() << ", ";
  os << "t0 = " << frame.bishopFrame.t.transpose() << ", ";
  os << "u = " << frame.bishopFrame.u.transpose() << ", ";
  os << "v = " << frame.bishopFrame.v.transpose() << ", ";
  os << "theta0 = " << frame.theta0 << ", ";
  os << "theta1 = " << frame.theta1;
  os << std::endl;
  return os;
}
