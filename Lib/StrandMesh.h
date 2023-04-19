#pragma once

#include <LibMath.h>

struct BishopFrame {
  // The tangent should always be defined
  Vec3<Real> t = Vec3<Real>::Zero();
  Vec3<Real> u = Vec3<Real>::Zero();
  Vec3<Real> v = Vec3<Real>::Zero();
};

/**
 * Rods make up the strands. Each rod is a collection of vertices. We assume
 * that edges are just vertex[i] to vertex[i+1]. The number of vertices must
 * always be at least 2.
 */
struct ModifiedBishopFrame {
  // These are the values controlling the bishop frame
  Vec3<Real> x0;
  Vec3<Real> x1;
  Vec3<Real> edge;

  // This is the bishop frame
  BishopFrame bishopFrame;

  // This is the "modified" portion, the theta
  Real theta;

  // Constructor which takes only x0 and x1
  ModifiedBishopFrame(const Vec3<Real> &x0, const Vec3<Real> &x1,
                      const Vec3<Real> &t, const Vec3<Real> &u,
                      const Vec3<Real> &v);
  friend auto operator<<(std::ostream &os, const ModifiedBishopFrame &frame)
      -> std::ostream &;
};

/**
 * This keeps track of the bending curvature of the material and is made up of
 * the individual points in the sim that make a stack of three.
 */
struct Bend {
  // The points that make up this bend.
  Vec3<Real> p0;
  Vec3<Real> p1;
  Vec3<Real> p2;

  // Discrete curvature binormal from equation 1 in Bergou et al.
  Vec3<Real> curvatureBinormal;

  // Material curvature from equation 2 in Bergou et al.
  Vec2<Real> gamma;
  Vec2<Real> prevGamma;

  Bend(const Vec3<Real> &p0, const Vec3<Real> &p1, const Vec3<Real> &p2,
       const Vec3<Real> &curvatureBinormal)
      : p0(p0), p1(p1), p2(p2), curvatureBinormal(curvatureBinormal) {
    gamma = Vec2<Real>::Zero();
    prevGamma = Vec2<Real>::Zero();
  }
};

class StrandMesh {
public:
  std::vector<ModifiedBishopFrame> rodSegments;
  std::vector<Bend> bendableSegments;

  // N x 1 boolean vector indicating whether a vertex is pinned. This matches
  // the dimension of v.
  Vec<int> pinned;

  explicit StrandMesh(const Mat<Real> &points);

  void Draw();
  void ComputeTotalEnergy();

  void UpdateQuasistaticFrame();
  void UpdateBishopFrames();
  void UpdateMaterialCurvature();

private:
  void WalkBishopFrames(const Mat<Real> &points);
};
