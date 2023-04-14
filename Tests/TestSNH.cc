#include <SNH.h>
#include <gtest/gtest.h>
#include <memory>

TEST(TestSNH, TestPK1Converges) {
  auto snhEnergy = std::make_unique<SNH>(1, 1);
  if (!snhEnergy->FiniteDifferenceTestPk1(Mat3<Real>::Random())) {
    FAIL();
  }
}

TEST(TestSNH, TestHessianConverges) {
  auto snhEnergy = std::make_unique<SNH>(1, 1);
  if (!snhEnergy->FiniteDifferenceTestHessian(Mat3<Real>::Random())) {
    FAIL();
  }
}