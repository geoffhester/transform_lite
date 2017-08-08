#include "gtest/gtest.h"

#include "constexprutil/linearalgebra/matrix.h"


TEST(MatrixMultiplication, MultiplyByIdentityReturnsSelf) {
  using constexprutil::linearalgebra::ConstexprMatrix;
  using constexprutil::linearalgebra::Identity;
  ConstexprMatrix<int,3,3> m{1,2,3,4,5,6,7,8,9};
  Identity<int,3> eye;

  ASSERT_NO_THROW(eye * m);
  auto result = eye * m;
  ASSERT_EQ(result,m);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}