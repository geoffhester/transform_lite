#include "gtest/gtest.h"

#include "constexprutil/linearalgebra/matrix.h"


TEST(MatrixMultiplication, MultiplyByIdentityReturnsSelf) {

}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}