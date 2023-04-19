#pragma once

#include <LibMath.h>

struct AdaptedFramedCurve {
  // Tangent vector
  Vec3<Real> t;

  // Normal vector
  Vec3<Real> m1;

  // Binormal vector
  Vec3<Real> m2;
};

class StrandMaterial {
public:
  StrandMaterial(Real bendingStiffness, Real twistingStiffness)
      : bendingStiffness(bendingStiffness),
        twistingStiffness(twistingStiffness) {}

  [[nodiscard]] INLINE auto Name() -> std::string { return "StrandMaterial"; }
  [[nodiscard]] auto TotalEnergy(const AdaptedFramedCurve &gamma,
                                 const AdaptedFramedCurve &restGamma) const -> Real;
  [[nodiscard]] auto BendingEnergy(const AdaptedFramedCurve &gamma,
                                   const AdaptedFramedCurve &restGamma) const
      -> Real;
  [[nodiscard]] auto TwistingEnergy(const AdaptedFramedCurve &gamma,
                                    const AdaptedFramedCurve &restGamma) const
      -> Real;

private:
  Real bendingStiffness;
  Real twistingStiffness;

  [[nodiscard]] auto DiscreteCurvatureBinormal(const Vec3<Real> &e0,
                                               const Vec3<Real> &e1) const
      -> Vec3<Real>;
  [[nodiscard]] auto MaterialCurvature(const Vec3<Real> &curvatureBinormal,
                                       const AdaptedFramedCurve &frame) const
      -> Vec2<Real>;
};
