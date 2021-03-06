#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "constexprutil/math/trigonometry.h"

int main() {
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

  for (auto x : thetas) {
    std::cout << std::setw(11) << std::setprecision(5) << x << " ";
  }
  std::cout << "\n\n";

  for(auto x : thetas) {
    std::cout << std::setw(11) << std::setprecision(5) << constexprutil::math::my_sin(x) << " ";
  }
  std::cout << '\n';
  for(auto x : thetas) {
    std::cout << std::setw(11) << std::setprecision(5) << sin(x) << " ";
  }
  std::cout << "\n\n";

  for(auto x : thetas) {
    std::cout << std::setw(11) << std::setprecision(5) << constexprutil::math::my_cos(x) << " ";
  }
  std::cout << '\n';
  for(auto x : thetas) {
    std::cout << std::setw(11) << std::setprecision(5) << cos(x) << " ";
  }
  std::cout << '\n';

  return 0;
}