#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "constexprutil/math/trigonometry.h"

std::vector<double> positive_pi_to_negative_pi_thetas() {
  std::vector<double const> const positive_fractions = {
      0.0,       1.0 / 6.0, 1.0 / 4.0, 1.0 / 3.0, 1.0 / 2.0,
      2.0 / 3.0, 3.0 / 4.0, 5.0 / 6.0, 1.0};

  std::vector<double> all_fractions;
  all_fractions.reserve(2 * positive_fractions.size());
  std::transform(std::begin(positive_fractions), std::end(positive_fractions),
                 std::back_inserter(all_fractions), [](auto x) { return -x; });
  std::reverse(std::begin(all_fractions), std::end(all_fractions));
  all_fractions.insert(std::end(all_fractions), std::begin(positive_fractions),
                       std::end(positive_fractions));

  std::vector<double> thetas;
  thetas.reserve(all_fractions.size());
  std::transform(std::begin(all_fractions), std::end(all_fractions),
                 std::back_inserter(thetas), [](auto x) { return M_PI * x; });

  return thetas;
}

TEST(SinTest, MatchesCoreLibrary) {
  std::vector<double> const thetas = positive_pi_to_negative_pi_thetas();

  using constexprutil::math::my_sin;

  for(auto& x : thetas) {
    ASSERT_NEAR(my_sin(x), sin(x), 1e-4);
  }
}

TEST(CosTest, MatchesCoreLibrary) {
  std::vector<double> const thetas = positive_pi_to_negative_pi_thetas();

  using constexprutil::math::my_cos;

  for(auto& x : thetas) {
    ASSERT_NEAR(my_cos(x), cos(x), 1e-4);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

