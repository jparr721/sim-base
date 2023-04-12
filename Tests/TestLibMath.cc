#include <LibMath.h>
#include <gtest/gtest.h>

TEST(TestLibMath, Flatten) {
  Mat2<Real> mat = Mat2<Real>::Random();
  Vec4<Real> comp;
  comp[0] = mat(0, 0);
  comp[1] = mat(1, 0);
  comp[2] = mat(0, 1);
  comp[3] = mat(1, 1);

  Vec4<Real> vec = Flatten(mat);
  EXPECT_EQ(vec, comp);
}

TEST(TestLibMath, UnFlatten) {
  Mat2<Real> mat = Mat2<Real>::Random();
  Vec4<Real> vec = Flatten(mat);
  Mat2<Real> comp = UnFlatten<2, 2>(vec);

  EXPECT_EQ(mat, comp);
}