#include <TetMesh.h>
#include <gtest/gtest.h>
#include <iostream>

Mat3<Real> I = Mat3<Real>::Identity();

TEST(TestTetMesh, TestPartialFPartialx0) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp.row(0) << -1, -1, -1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(0, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx1) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp.row(1) << -1, -1, -1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(1, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx2) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp.row(2) << -1, -1, -1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(2, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx3) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(0, 0) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(3, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx4) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(1, 0) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(4, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx5) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(2, 0) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(5, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx6) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(0, 1) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(6, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx7) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(1, 1) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(7, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx8) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(2, 1) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(8, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx9) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(0, 2) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(9, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx10) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(1, 2) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(10, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx11) {
  Mat3<Real> comp = Mat3<Real>::Zero();
  comp(2, 2) = 1;
  Mat3<Real> partial = TetMesh<Real>::EvalPartialFPartialx(11, I);
  EXPECT_EQ(partial, comp);
}

TEST(TestTetMesh, TestPartialFPartialx) {
  Mat9x12<Real> comp = Mat9x12<Real>::Zero();
  comp << -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0,
      0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  Mat9x12<Real> ret = TetMesh<Real>::PartialFPartialx(I);
  EXPECT_EQ(ret, comp);
}