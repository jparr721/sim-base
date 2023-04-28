#include <LibMath.h>
#include <gtest/gtest.h>

TEST(TestLibMath, ColwiseFlatten) {
  Mat2<Real> mat;
  mat << 1, 3, 2, 4;
  Vec4<Real> comp;
  comp[0] = mat(0, 0);
  comp[1] = mat(1, 0);
  comp[2] = mat(0, 1);
  comp[3] = mat(1, 1);

  Vec4<Real> vec = ColwiseFlatten<Real>(mat);
  EXPECT_EQ(vec, comp);
}

TEST(TestLibMath, ColwiseUnFlatten) {
  Mat2<Real> mat;
  mat << 1, 3, 2, 4;
  Vec4<Real> vec = ColwiseFlatten<Real>(mat);
  Mat2<Real> comp = ColwiseUnFlatten<Real>(vec, 2, 2);

  EXPECT_EQ(mat, comp);

  Mat9x12<Real> bigger = Mat9x12<Real>::Random();

  EXPECT_EQ(bigger,
            ColwiseUnFlatten<Real>(ColwiseFlatten<Real>(bigger), 9, 12));
}

TEST(TestLibMath, RowwiseFlatten) {
  Mat2<Real> mat;
  mat << 1, 3, 2, 4;
  Vec4<Real> comp;
  comp[0] = mat(0, 0);
  comp[1] = mat(0, 1);
  comp[2] = mat(1, 0);
  comp[3] = mat(1, 1);

  Vec4<Real> vec = RowwiseFlatten<Real>(mat);
  EXPECT_EQ(vec, comp);
}

TEST(TestLibMath, RowwiseUnFlatten) {
  Mat2<Real> mat;
  mat << 1, 3, 2, 4;
  Vec4<Real> vec = RowwiseFlatten<Real>(mat);
  Mat2<Real> comp = RowwiseUnFlatten<Real>(vec, 2, 2);

  EXPECT_EQ(mat, comp);

  Mat9x12<Real> bigger = Mat9x12<Real>::Random();

  EXPECT_EQ(bigger,
            RowwiseUnFlatten<Real>(RowwiseFlatten<Real>(bigger), 9, 12));
}

TEST(TestLibMath, MakeTriDiagonalMatrix) {
  Vec<Real> center(5);
  center << 1, 1, 1, 1, 1;

  Vec<Real> upper(4);
  upper << 2, 2, 2, 2;

  Vec<Real> lower(4);
  lower << 3, 3, 3, 3;

  SparseMat<Real> triDiagonal = MakeTriDiagonalMatrix(upper, center, lower);

  EXPECT_EQ(triDiagonal.coeff(0, 0), 1);
  for (int ii = 1; ii < 5; ++ii) {
    EXPECT_EQ(triDiagonal.coeff(ii, ii), 1);
    if (ii < 4) {
      EXPECT_EQ(triDiagonal.coeff(ii, ii + 1), 2);
    }
    EXPECT_EQ(triDiagonal.coeff(ii, ii - 1), 3);
  }
}

TEST(TestLibMath, FactorTriDiagonalMatrix) {
  for (int ii = 0; ii < 10; ++ii) {

    Vec<Real> center = Vec<Real>::Random(5);
    Vec<Real> upper = Vec<Real>::Random(4);
    Vec<Real> lower = Vec<Real>::Random(4);
    Vec<Real> b = Vec<Real>::Random(5);

    Vec<Real> x = FactorTriDiagonalMatrix(upper, center, lower, b);
    SparseMat<Real> tri = MakeTriDiagonalMatrix(upper, center, lower);
    Eigen::SparseLU<SparseMat<Real>> luSolver(tri);
    Vec<Real> x2 = luSolver.solve(b);

    EXPECT_TRUE(x.isApprox(x2));
  }
}