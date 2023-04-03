#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

using Real = double;
using Float = float;

constexpr Real PI = Real(3.14159265);
constexpr Real THREE_QUARTER_PI = Real(2.0944);
constexpr Real HALF_PI = Real(1.57079633);
constexpr Real QUARTER_PI = Real(0.7853981625);
constexpr Real E = Real(2.71828182845904523536);

constexpr Real DEGREE_180 = Real(180);
constexpr Real DegToRadConv = Real(PI) / DEGREE_180;
constexpr Real RadToDegConv = DEGREE_180 / Real(PI);

enum Axis { X, Y, Z, NX, NY, NZ, NO_AXIS };
enum class AxisPlane { XY, YZ, XZ, NO_AXISPLANE };
enum Dimension { DIM1 = 1, DIM2 = 2, DIM3 = 3, DIM4 = 4 };

template <typename Real> inline auto DegreesToRadians(Real degrees) -> Real {
  return degrees * Real(DegToRadConv);
}

template <typename Real> inline auto RadiansToDegrees(Real radians) -> Real {
  return radians * Real(RadToDegConv);
}

template <typename Real> using Vec2 = Eigen::Matrix<Real, 2, 1>;

template <typename Real> using Normal2 = Eigen::Matrix<Real, 2, 1>;

template <typename Real> using Vec3 = Eigen::Matrix<Real, 3, 1>;

template <typename Real> using Normal3 = Eigen::Matrix<Real, 3, 1>;

template <typename Real> using Vec4 = Eigen::Matrix<Real, 4, 1>;

template <typename Real> using Normal4 = Eigen::Matrix<Real, 4, 1>;

template <typename Real> using Mat2 = Eigen::Matrix<Real, 2, 2>;

template <typename Real> using Mat3 = Eigen::Matrix<Real, 3, 3>;

template <typename Real> using Vec = Eigen::Matrix<Real, Eigen::Dynamic, 1>;

template <typename Real> using Matrix4 = Eigen::Matrix<Real, 4, 4>;

template <typename Real>
using Mat = Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Real> using Quaternion = Eigen::Quaternion<Real>;

template <typename Real> using AngleAxis = Eigen::AngleAxis<Real>;

template <typename Real>
inline auto Normalize(const Real &value, const Real &min, const Real &max)
    -> Real {
  return (value - min) / (max - min);
}

template <typename Real>
inline auto Denormalize(const Real &value, const Real &min, const Real &max)
    -> Real {
  return value * (max - min) + min;
}

template <typename Real>
inline auto ComputeAverage(const std::vector<Real> &values) -> Real {
  Real sum = Real(0);
  std::size_t n = values.size();
  for (std::size_t i = 0; i < n; i++)
    sum += values[i];
  return sum / Real(n);
}

template <typename Real>
auto Equivalent(const Real &a, const Real &b,
                const Real &epsilon = Real(1.0e-4)) -> bool {
  if (a >= b - epsilon && a <= b + epsilon)
    return true;
  return false;
}

template <typename Real>
auto Equivalent(const Vec2<Real> &a, const Vec2<Real> &b,
                Real epsilon = Real(1.0e-4)) -> bool {
  if (a.x() < b.x() - epsilon || a.x() > b.x() + epsilon)
    return false;
  if (a.y() < b.y() - epsilon || a.y() > b.y() + epsilon)
    return false;
  return true;
}

template <typename Real>
auto Equivalent(const Vec3<Real> &a, const Vec3<Real> &b,
                Real epsilon = Real(1.0e-4)) -> bool {
  if (a.x() < b.x() - epsilon || a.x() > b.x() + epsilon)
    return false;
  if (a.y() < b.y() - epsilon || a.y() > b.y() + epsilon)
    return false;
  if (a.z() < b.z() - epsilon || a.z() > b.z() + epsilon)
    return false;
  return true;
}

template <typename Real>
inline auto AngleBetweenVectors(const Vec2<Real> &a, const Vec2<Real> &b)
    -> Real {
  return std::acos((a.x() * b.x() + a.y() * b.y()) / (a.norm() * b.norm()));
}

template <typename Real>
inline auto AngleBetweenVectors(const Vec3<Real> &a, const Vec3<Real> &b)
    -> Real {
  return std::acos((a.x() * b.x() + a.y() * b.y() + a.z() * b.z()) /
                   (a.norm() * b.norm()));
}

template <typename Real>
auto Distance(const Vec2<Real> &a, const Vec2<Real> &b) -> Real {
  Real dx = b.x() - a.x();
  Real dy = b.y() - a.y();
  return std::sqrt(dx * dx + dy * dy);
}

template <typename Real>
auto Distance(const Vec3<Real> &a, const Vec3<Real> &b) -> Real {
  Real dx = b.x() - a.x();
  Real dy = b.y() - a.y();
  Real dz = b.z() - a.z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/*
 * Scales the provided value v, which resides within the source interval:
 * [sourceMin, sourceMax] to its respective location within the target
 * interval: [targetMin, targetMax]. If the intervals are invalid or the
 * provided value does not reside within the source interval this function
 * will return false; otherwise it will return true.
 *
 * @param v - The value to be modified (scaled)
 * @param sourceMin - Lower limit of the source interval
 * @param sourceMax - Upper limit of the source interval
 * @param targetMin - Lower limit of the target interval
 * @param targetMax - Upper limit of the target interval
 * @return success
 *
 * Usuage:
 * double value = 50.0;
 * px::Scale(value, 0.0, 100.0, 0.0, 1.0);
 *
 * Result:
 * value = 0.5
 */
template <typename Real>
inline auto Scale(Real &v, Real sourceMin, Real sourceMax, Real targetMin,
                  Real targetMax) -> bool {
  Real l = (targetMax - targetMin);
  Real m = l / Real(sourceMax - sourceMin);
  if (l == static_cast<Real>(0))
    m = static_cast<Real>(0);
  v = targetMin + ((v - sourceMin) * m);
  return true;
}

/*
 * Scales an entire array of values included within the source interval:
 * [sourceMin, sourceMax] to their respective values within the target interval:
 * [targetMin, targetMax]. If each value is correctly scaled this function
 * returns true; otherwise, if any value cannot be scaled the algorithm halts
 * and returns false.
 *
 * @param array - The pointer to the value array
 * @param n - The length of the input array
 * @param sourceMin - Lower limit of the source interval
 * @param sourceMax - Upper limit of the source interval
 * @param targetMin - Lower limit of the target interval
 * @param targetMax - Upper limit of the target interval
 * @return success
 *
 * Usage:
 * const unsigned int n = 4;
 * double* values = new double[n + 1];
 * for ( unsigned int i = 0; i < n + 1; i++ )
 *    values[i] = static_cast<double>(i);
 * px::Scale(values, n + 1, 0.0, 4.0, 0.0, 1.0);
 * for ( unsigned int i = 0; i < n + 1; i++ )
 *    std::cout << values[i] << std::endl;
 * delete [] values;
 *
 * Result:
 * values = 0.0, 0.25, 0.5, 0.75, 1.0
 */
template <typename Real>
inline auto ScaleArray(Real *&array, unsigned int n, Real sourceMin,
                       Real sourceMax, Real targetMin, Real targetMax) -> bool {
  if (array == nullptr)
    return false;
  if (n <= 1)
    return false;

  for (unsigned int i = 0; i < n; i++) {
    bool success = Scale(array[i], sourceMin, sourceMax, targetMin, targetMax);
    if (!success)
      return false;
  }

  return true;
}

template <typename Real>
auto CartesianToSpherical(const Vec3<Real> &cartesianCoord)
    -> Vec3<Real> {
  Vec3<Real> sphericalCoord;

  Real x = cartesianCoord.x();
  Real z = cartesianCoord.y();
  Real y = cartesianCoord.z();

  Real r = std::sqrt(x * x + y * y + z * z);
  Real theta = std::atan2(y, x);
  Real phi = std::acos(z / r);

  sphericalCoord.x() = r;
  sphericalCoord.y() = theta;
  sphericalCoord.z() = phi;

  return sphericalCoord;
}

template <typename Real>
auto SphericalToCartesian(const Vec3<Real> &sphericalCoord)
    -> Vec3<Real> {
  Vec3<Real> cartesianCoord;
  Real r = sphericalCoord.x();
  Real theta = sphericalCoord.z();
  Real phi = sphericalCoord.y();

  Real cosTheta = std::cos(theta);
  Real cosPhi = std::cos(phi);
  Real sinTheta = std::sin(theta);
  Real sinPhi = std::sin(phi);

  cartesianCoord.x() = r * (cosTheta * sinPhi);
  cartesianCoord.y() = r * (sinTheta * sinPhi);
  cartesianCoord.z() = r * cosPhi;

  return cartesianCoord;
}

template <typename T, typename Derived>
inline void HStack(const std::vector<T> &values,
                   Eigen::PlainObjectBase<Derived> &stacked) {
  if (values.size() == 0) {
    return;
  }
  const uint64_t cols = values.at(0).transpose().cols();
  const uint64_t rows = values.size();

  stacked.resize(rows, cols);
  for (auto i = 0u; i < values.size(); ++i) {
    stacked.row(i) = values.at(i);
  }
}

template <typename T, typename Fn, typename Derived>
inline void HStack(const std::vector<T> &values, const Fn &extractor,
                   Eigen::PlainObjectBase<Derived> &stacked) {
  if (values.size() == 0) {
    return;
  }
  const uint64_t cols = extractor(values.at(0)).transpose().cols();
  const uint64_t rows = values.size();

  stacked.resize(rows, cols);
  for (auto i = 0u; i < values.size(); ++i) {
    stacked.row(i) = extractor(values.at(i));
  }
}

template <typename DerivedIn, typename DerivedOut>
inline void Unstack(const Eigen::MatrixBase<DerivedIn> &stacked,
                    std::vector<Vec3<DerivedOut>> &rows) {
  rows.resize(stacked.rows());
  for (int row = 0; row < stacked.rows(); ++row) {
    rows.at(row) = stacked.row(row);
  }
}

template <typename Derived>
inline void MatrixToVector(const Eigen::MatrixBase<Derived> &in,
                           Eigen::PlainObjectBase<Derived> &out) {
  const typename Derived::Scalar *data = in.data();
  const auto shape = in.rows() * in.cols();
  out = Eigen::Map<Eigen::PlainObjectBase<Derived>>(data, shape);
}

template <typename T>
inline auto IsApprox(const T &lhs, const T &rhs, const Real epsilon) -> bool {
  return std::abs(lhs - rhs) < epsilon;
}

template <typename T>
inline auto CloserTo(const T &lhs, const T &rhs, const T &value) -> T {
  const auto labsdiff = std::abs(lhs - value);
  const auto rabsdiff = std::abs(rhs - value);

  return labsdiff > rabsdiff ? rhs : lhs;
}

template <typename T>
inline auto PointIsOnLine(const Vec2<T> &point, const Vec2<T> &start,
                          const Vec3<T> &direction) -> bool {
  return direction.cross(point - start) == Vec2<T>::Zero();
}

inline auto PointLineDistanceSquared3D(const Vec3<Real> &point,
                                       const Vec3<Real> &start,
                                       const Vec3<Real> &direction, Real &t)
    -> Real {
  t = (direction.dot(point - start) / direction.dot(direction));
  const Vec3<Real> pointPrime = start + t * direction;
  const Vec3<Real> rightAngle = point - pointPrime;
  const Real distanceSquared = rightAngle.dot(rightAngle);
  return distanceSquared;
}

inline auto PointRayDistanceSquared3D(const Vec3<Real> &point,
                                      const Vec3<Real> &start,
                                      const Vec3<Real> &direction, Real &t)
    -> Real {
  Real distanceSquared;
  distanceSquared = PointLineDistanceSquared3D(point, start, direction, t);

  if (t < 0) {
    t = 0;
    Vec3<Real> v = point - start;
    distanceSquared = v.dot(v);
  }

  return distanceSquared;
}

template <typename T>
inline auto JointPointPointAngleRad(const Vec3<T> &jointPos,
                                    const Vec3<T> &neighborOnePos,
                                    const Vec3<T> &neighborTwoPos) -> float {
  const Vec3<Real> neighborOneDir = (neighborOnePos - jointPos).normalized();
  const Vec3<Real> neighborTwoDir = (neighborTwoPos - jointPos).normalized();
  return AngleBetweenVectors(neighborOneDir, neighborTwoDir);
}