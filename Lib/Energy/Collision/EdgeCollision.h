#pragma once
#include <LibMath.h>

///////////////////////////////////////////////////////////////////////////////////
// This is (sort of) the edge-edge collision energy described in a variety of
// places:
//
// "Dynamic deformables: implementation and production practicalities"
// Kim and Eberle, 2020, Chapter 11.4
//
// "Robust treatement of simultaneous contacts"
// Harmon et al., 2008, Section 2
//
// "Collision and self-collision handling in cloth model dedicated to design
// garments" Provot 1997, Section 3.3
//
// I haven't actually seen it written down as an energy anywhere. The Harmon
// paper comes closest by posing an unsquared constraint.
//
// The energy is extremely similar to the vertex-face collision energy from:
//
// "Efficient elasticity for character skinning with contact and collisions"
// McAdams et al. 2011.
//
// The normal gradient and Hessian stuff works out to almost the same thing,
// up to a few constants
//
// vertex and coordinate ordering is:
//
//    vector<Vec3<Real>> vs(4);
//    vs[0] = _vertices[edge0[0]];
//    vs[1] = _vertices[edge0[1]];
//    vs[2] = _vertices[edge1[0]];
//    vs[3] = _vertices[edge1[1]];
//
//    const Vec2<Real>& a = _edgeEdgeCoordinates[i].first;
//    const Vec2<Real>& b = _edgeEdgeCoordinates[i].second;
//
///////////////////////////////////////////////////////////////////////////////////
class EdgeCollision {
public:
  EdgeCollision(const Real &mu, const Real &eps = 0.0);
  virtual ~EdgeCollision(){};

  // get the strain energy
  Real psi(const std::vector<Vec3<Real>> &v, const Vec2<Real> &a,
           const Vec2<Real> &b) const;
  virtual Real psi(const Vec12<Real> &x, const Vec2<Real> &a,
                   const Vec2<Real> &b) const;

  // negated version of strain energy, if it's already intersecting
  Real psiNegated(const std::vector<Vec3<Real>> &v, const Vec2<Real> &a,
                  const Vec2<Real> &b) const;
  virtual Real psiNegated(const Vec12<Real> &x, const Vec2<Real> &a,
                          const Vec2<Real> &b) const;

  // This is the *gradient* of psi. The force is the *negative* gradient of psi.
  Vec12<Real> gradient(const std::vector<Vec3<Real>> &v, const Vec2<Real> &a,
                       const Vec2<Real> &b) const;
  virtual Vec12<Real> gradient(const Vec12<Real> &x, const Vec2<Real> &a,
                               const Vec2<Real> &b) const;

  // negated version of gradient, if it's already intersecting
  Vec12<Real> gradientNegated(const std::vector<Vec3<Real>> &v,
                              const Vec2<Real> &a, const Vec2<Real> &b) const;
  virtual Vec12<Real> gradientNegated(const Vec12<Real> &x, const Vec2<Real> &a,
                                      const Vec2<Real> &b) const;

  const Real &mu() const { return _mu; };
  const Real &eps() const { return _eps; };
  Real &mu() { return _mu; };
  // Real& eps() { return _eps; };
  virtual void setEps(const Real &eps) { _eps = eps; };

  // convert vertices to edges before determining if the two
  // edges are nearly parallel. this is needed by unit tests to
  // determine if we should give up on the current test.
  static bool nearlyParallelVertices(const std::vector<Vec3<Real>> &x);

protected:
  // convert the 12-vector in a way that imposes a consistent tet
  // ordering for vertices and edges
  static void getVerticesAndEdges(const Vec12<Real> &x,
                                  std::vector<Vec3<Real>> &v,
                                  std::vector<Vec3<Real>> &e);

  // gradient of spring length, n' * (va - vb)
  static Vec12<Real> springLengthGradient(const std::vector<Vec3<Real>> &e,
                                          const Vec3<Real> &n,
                                          const Vec3<Real> &diff,
                                          const Vec2<Real> &a,
                                          const Vec2<Real> &b);

  // hessian of spring length, n' * (va - vb)
  static Mat12<Real> springLengthHessian(const std::vector<Vec3<Real>> &e,
                                         const Vec3<Real> &n,
                                         const Vec3<Real> &diff,
                                         const Vec2<Real> &a,
                                         const Vec2<Real> &b);

  // partial of (va - vb)
  static Mat3x12<Real> vDiffPartial(const Vec2<Real> &a, const Vec2<Real> &b);

  // are the two edges nearly parallel?
  static bool nearlyParallelEdges(const std::vector<Vec3<Real>> &e);

  // collision stiffness
  Real _mu;

  // collision epsilon -- how far apart should we push things?
  Real _eps;
};
