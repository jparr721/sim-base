#include "VertexFaceSqrtCollision.h"

VertexFaceSqrtCollision::VertexFaceSqrtCollision(Real collisionStiffness,
                                                 Real collisionEpsilon)
    : collisionStiffness(collisionStiffness),
      collisionEpsilon(collisionEpsilon) {}

auto VertexFaceSqrtCollision::Gradient(const Vec12<Real> &x) const
    -> Vec12<Real> {
  constexpr Real inverseEps = 1e-8;
  std::vector<Vec3<Real>> v;
  std::vector<Vec3<Real>> e;
  GetVerticesAndEdges(x, v, e);
  const Vec3<Real> bary = GetInsideBarycentricCoordinates(v);
  const auto reversal = ShouldReverse(v, e);

  // remember we had to reorder vertices in a wonky way
  const Vec3<Real> xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
  const Vec3<Real> t = v[0] - xs;
  const Real tMagnitude = sqrt(t.dot(t));
  // const Real springDiff = tMagnitude - _eps;
  const Real springDiff = (reversal) ? tMagnitude + collisionEpsilon
                                     : tMagnitude - collisionEpsilon;
  const Mat3x12<Real> tDiff = tDiffPartial(bary);

  // if everything has become undefined, just give up
  const Real tDott = t.dot(t);
  if (fabs(tMagnitude) <= inverseEps || fabs(tDott) < inverseEps)
    return Vec12<Real>::Zero();

  Vec12<Real> result = 2.0 * collisionStiffness * springDiff *
                       (1.0 / tMagnitude) * tDiff.transpose() * t;
  return result;
}

auto VertexFaceSqrtCollision::GetInsideBarycentricCoordinates(
    const std::vector<Vec3<Real>> &vertices) const -> Vec3<Real> {
  Vec3<Real> barycentric = GetBarycentricCoordinates(vertices);

  // if it's already inside, we're all done
  if (barycentric[0] >= 0.0 && barycentric[1] >= 0.0 && barycentric[2] >= 0.0) {
    return barycentric;
  }

  // find distance to all the line segments
  //
  // there's lots of redundant computation between here and getLerp,
  // but let's get it working and see if it fixes the actual
  // artifact before optimizing
  Real distance12 = PointLineDistance(vertices[0], vertices[1], vertices[2]);
  Real distance23 = PointLineDistance(vertices[0], vertices[2], vertices[3]);
  Real distance31 = PointLineDistance(vertices[0], vertices[3], vertices[1]);

  // less than or equal is important here, otherwise fallthrough breaks
  if (distance12 <= distance23 && distance12 <= distance31) {
    Vec2<Real> lerp = GetLerp(vertices[0], vertices[1], vertices[2]);
    barycentric[0] = lerp[0];
    barycentric[1] = lerp[1];
    barycentric[2] = 0.0;
    return barycentric;
  }

  // less than or equal is important here, otherwise fallthrough breaks
  if (distance23 <= distance12 && distance23 <= distance31) {
    Vec2<Real> lerp = GetLerp(vertices[0], vertices[2], vertices[3]);
    barycentric[0] = 0.0;
    barycentric[1] = lerp[0];
    barycentric[2] = lerp[1];
    return barycentric;
  }

  // else it must be the 31 case
  Vec2<Real> lerp = GetLerp(vertices[0], vertices[3], vertices[1]);
  barycentric[0] = lerp[1];
  barycentric[1] = 0.0;
  barycentric[2] = lerp[0];
  return barycentric;
}

auto VertexFaceSqrtCollision::tDiffPartial(const Vec3<Real> &bary) const
    -> Mat3x12<Real> {
  Mat3x12<Real> tPartial;
  tPartial.setZero();
  tPartial(0, 0) = tPartial(1, 1) = tPartial(2, 2) = 1.0;
  tPartial(0, 3) = tPartial(1, 4) = tPartial(2, 5) = -bary[0];
  tPartial(0, 6) = tPartial(1, 7) = tPartial(2, 8) = -bary[1];
  tPartial(0, 9) = tPartial(1, 10) = tPartial(2, 11) = -bary[2];

  return tPartial;
}
