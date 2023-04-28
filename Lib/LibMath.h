#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Settings.h>
#include <cmath>
#include <iostream>
#include <random>
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
template <typename Real> using Mat2x3 = Eigen::Matrix<Real, 2, 3>;
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

template <typename T>
INLINE auto AngleBetweenVectors(const Vec3<T> &a, const Vec3<T> &b) -> T {
  return std::acos(a.dot(b) / (a.norm() * b.norm()));
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

template <typename T>
INLINE auto ConstructDiagonalSparseMatrix(const Vec<T> &v) -> SparseMat<T> {
  int n = v.size();
  SparseMat<T> matrix(n, n);
  for (int i = 0; i < n; ++i) {
    matrix.insert(i, i) = v(i);
  }
  matrix.makeCompressed();
  return matrix;
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

INLINE auto GetOrthogonalVectors(const Vec3<Real> &v)
    -> std::pair<Vec3<Real>, Vec3<Real>> {
  Vec3<Real> u = v.normalized();
  Vec3<Real> r = Vec3<Real>::Random();
  double d = r.dot(u);
  r -= d * u;
  Vec3<Real> w = r.normalized();
  Vec3<Real> t = u.cross(w);
  return {w, t};
}

INLINE auto GetOrthogonalVector(const Vec3<Real> &u) -> Vec<Real> {
  ASSERT(u.norm() != 0, "Degenerate normal found");

  Vec3<Real> v = Vec3<Real>::Zero();
  int max = 0;
  for (int i = 0; i < u.size(); ++i) {
    if (u[i] == 0) {
      v[i] = 1;
      return v;
    }
    if (fabs(u[i]) > fabs(u[max])) {
      max = i;
    }
  }

  int idx = (max + 1) % u.size();
  v[idx] = u[max];
  v[max] = -u[idx];
  v.normalize();

  ASSERT2(std::abs(u.dot(v)) < 1e-10);

  return v;
}

INLINE auto ParallelTransport(const Vec3<Real> &t, const Vec3<Real> &u,
                              const Vec3<Real> &v) -> Vec3<Real> {
  // Compute the bitangent of u and v
  Vec3<Real> bitangent = u.cross(v);

  // If it is the zero vector, u and v are the same value.
  if (bitangent.norm() == 0) {
    return t;
  }

  // Normalize
  bitangent.normalize();

  // Remove the similarity between the first u and the bitangent
  bitangent = (bitangent - (bitangent.dot(u)) * u).normalized();

  // Do the same with v.
  bitangent = (bitangent - (bitangent.dot(v)) * v).normalized();

  // Get two orthogonal vectors to u and v
  const Vec3<Real> n1 = u.cross(bitangent);
  const Vec3<Real> n2 = v.cross(bitangent);

  return t.dot(u) * v + t.dot(n1) * n2 + t.dot(bitangent) * bitangent;
}

INLINE auto GenerateRandomNumber01() -> Real {
  // Create a random number generator engine
  static std::default_random_engine generator;

  // Create a distribution for generating random numbers in the range [0, 1]
  static std::uniform_real_distribution<Real> distribution(0.0, 1.0);

  // Generate a random number and return it
  return distribution(generator);
}

INLINE auto CrossProductMatrix(Real a, Real b, Real c) -> Mat3<Real> {
  Mat3<Real> cross = Mat3<Real>::Zero();
  cross(0, 1) = -c;
  cross(0, 2) = b;
  cross(1, 0) = c;
  cross(1, 2) = -a;
  cross(2, 0) = -b;
  cross(2, 1) = a;
  return cross;
}

INLINE auto CrossProductMatrix(const Vec3<Real> &v) -> Mat3<Real> {
  return CrossProductMatrix(v(0), v(1), v(2));
}

INLINE auto MakeTriDiagonalMatrix(const Vec<Real> &upper,
                                  const Vec<Real> &center,
                                  const Vec<Real> &lower) -> SparseMat<Real> {
  const int n = center.size();
  SparseMat<Real> A(n, n);

  for (int i = 0; i < n; ++i) {
    if (i > 0) {
      A.insert(i, i - 1) = lower(i - 1);
    }

    A.insert(i, i) = center(i);

    if (i < n - 1) {
      A.insert(i, i + 1) = upper(i);
    }
  }

  A.makeCompressed();
  return A;
}

INLINE auto FactorTriDiagonalMatrix(const Vec<Real> &upper,
                                    const Vec<Real> &center,
                                    const Vec<Real> &lower, const Vec<Real> &b)
    -> Vec<Real> {
  SparseMat<Real> tri = MakeTriDiagonalMatrix(upper, center, lower);
  Eigen::SparseLU<SparseMat<Real>> luSolver(tri);
  return luSolver.solve(b);
}