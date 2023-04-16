#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Settings.h>
#include <cmath>
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

template <typename Real> INLINE auto DegreesToRadians(Real degrees) -> Real {
  return degrees * Real(DegToRadConv);
}

template <typename Real> INLINE auto RadiansToDegrees(Real radians) -> Real {
  return radians * Real(RadToDegConv);
}

template <typename Real> using Vec2 = Eigen::Matrix<Real, 2, 1>;
template <typename Real> using Vec3 = Eigen::Matrix<Real, 3, 1>;
template <typename Real> using Vec4 = Eigen::Matrix<Real, 4, 1>;
template <typename Real> using Vec9 = Eigen::Matrix<Real, 9, 1>;
template <typename Real> using Vec12 = Eigen::Matrix<Real, 12, 1>;
template <typename Real> using Vec = Eigen::Matrix<Real, -1, 1>;

template <typename Real> using Mat2 = Eigen::Matrix<Real, 2, 2>;
template <typename Real> using Mat3 = Eigen::Matrix<Real, 3, 3>;
template <typename Real> using Mat4 = Eigen::Matrix<Real, 4, 4>;
template <typename Real> using Mat9 = Eigen::Matrix<Real, 9, 9>;
template <typename Real> using Mat9x12 = Eigen::Matrix<Real, 9, 12>;
template <typename Real> using Mat = Eigen::Matrix<Real, -1, -1>;

template <typename Real> using SparseMat = Eigen::SparseMatrix<Real>;

template <typename Real>
INLINE auto Normalize(const Real &value, const Real &min, const Real &max)
    -> Real {
  return (value - min) / (max - min);
}

template <typename Real>
INLINE auto Denormalize(const Real &value, const Real &min, const Real &max)
    -> Real {
  return value * (max - min) + min;
}

template <typename Real>
INLINE auto ComputeAverage(const std::vector<Real> &values) -> Real {
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
INLINE auto AngleBetweenVectors(const Vec2<Real> &a, const Vec2<Real> &b)
    -> Real {
  return std::acos((a.x() * b.x() + a.y() * b.y()) / (a.norm() * b.norm()));
}

template <typename Real>
INLINE auto AngleBetweenVectors(const Vec3<Real> &a, const Vec3<Real> &b)
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
INLINE auto Scale(Real &v, Real sourceMin, Real sourceMax, Real targetMin,
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
INLINE auto ScaleArray(Real *&array, unsigned int n, Real sourceMin,
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

template <typename T> INLINE auto ColwiseFlatten(const Mat<T> &mat) -> Vec<T> {
  Vec<T> flattened(mat.size());
  unsigned int index = 0;
  for (int jj = 0; jj < mat.cols(); ++jj) {
    for (int ii = 0; ii < mat.rows(); ++ii, ++index) {
      flattened(index) = mat(ii, jj);
    }
  }
  return flattened;
}

template <typename T>
INLINE auto ColwiseUnFlatten(const Vec<T> &x, int rows, int cols) -> Mat<T> {
  ASSERT(x.size() == rows * cols, "x.size(): " + std::to_string(x.size()) +
                                      " rows, cols " + std::to_string(rows) +
                                      " " + std::to_string(cols));
  Mat<T> ret(rows, cols);
  unsigned int index = 0;
  for (int jj = 0; jj < cols; ++jj) {
    for (int ii = 0; ii < rows; ++ii, ++index) {
      ret(ii, jj) = x(index);
    }
  }
  return ret;
}

template <typename T> INLINE auto RowwiseFlatten(const Mat<T> &mat) -> Vec<T> {
  Vec<T> flattened(mat.size());
  unsigned int index = 0;
  for (int ii = 0; ii < mat.rows(); ++ii) {
    for (int jj = 0; jj < mat.cols(); ++jj, ++index) {
      flattened(index) = mat(ii, jj);
    }
  }
  return flattened;
}

template <typename T>
INLINE auto RowwiseUnFlatten(const Vec<T> &x, int rows, int cols) -> Mat<T> {
  ASSERT(x.size() == rows * cols, "x.size(): " + std::to_string(x.size()) +
                                      " rows, cols " + std::to_string(rows) +
                                      " " + std::to_string(cols));
  Mat<T> ret(rows, cols);
  unsigned int index = 0;
  for (int ii = 0; ii < rows; ++ii) {
    for (int jj = 0; jj < cols; ++jj, ++index) {
      ret(ii, jj) = x(index);
    }
  }
  return ret;
}

template <typename T>
INLINE auto IsApprox(const T &lhs, const T &rhs, const Real epsilon) -> bool {
  return std::abs(lhs - rhs) < epsilon;
}

template <typename T>
INLINE auto CloserTo(const T &lhs, const T &rhs, const T &value) -> T {
  const auto labsdiff = std::abs(lhs - value);
  const auto rabsdiff = std::abs(rhs - value);

  return labsdiff > rabsdiff ? rhs : lhs;
}