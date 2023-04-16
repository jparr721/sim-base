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
