#ifndef TRANSFORM_LITE_TRIGONOMETRY_H
#define TRANSFORM_LITE_TRIGONOMETRY_H

#include <cmath>
#include <stdexcept>

namespace constexprutil {
namespace math {

constexpr double my_sin_impl(double x, double current_intermediate, int i,
                    int i_max) {
  current_intermediate *= -(x * x) / ((2 * i - 1) * (2 * i - 2));

  double ret =  current_intermediate +
      (i < (i_max - 1) ? my_sin_impl(x, current_intermediate, ++i, i_max)
                       : x);
  return ret;
}

constexpr double my_sin(double x) {
  int const num_iter = 10;
  if (x == 0.0 || x == M_PI || x == -M_PI) {
    return 0.0;
  }
  if (0.0 <= x && x <= M_PI_2) {
    return my_sin_impl(x, x, 2, num_iter);
  }
  if (M_PI_2 < x ) {
    return my_sin(M_PI - x);
  }
  // x < 0.0
  return -my_sin(-x);
}

constexpr double my_cos(double x) {
  return my_sin(x + M_PI_2);
}

}  // namespace math
}  // namespace constexprutil

#endif //TRANSFORM_LITE_TRIGONOMETRY_H
