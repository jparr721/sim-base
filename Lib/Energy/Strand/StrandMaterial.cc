#include <Energy/Strand/StrandMaterial.h>

auto StrandMaterial::TotalEnergy(const AdaptedFramedCurve &gamma,
                                 const AdaptedFramedCurve &restGamma) const
    -> Real {
  return BendingEnergy(gamma, restGamma) + TwistingEnergy(gamma, restGamma);
}

auto StrandMaterial::BendingEnergy(const AdaptedFramedCurve &gamma,
                                   const AdaptedFramedCurve &restGamma) const
    -> Real {

}

auto StrandMaterial::TwistingEnergy(const AdaptedFramedCurve &gamma,
                                    const AdaptedFramedCurve &restGamma) const
    -> Real {
  return 0;
}

auto StrandMaterial::DiscreteCurvatureBinormal(const Vec3<Real> &e0,
                                               const Vec3<Real> &e1) const
    -> Vec3<Real> {
  return (2.0 * e0.cross(e1)) / (e0.dot(e1) + e0.norm() * e1.norm());
}

auto StrandMaterial::MaterialCurvature(const Vec3<Real> &curvatureBinormal,
                                       const AdaptedFramedCurve &frame) const
    -> Vec2<Real> {
  return {curvatureBinormal.dot(frame.m2), -curvatureBinormal.dot(frame.m1)};
}
