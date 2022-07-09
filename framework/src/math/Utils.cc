#include "../../inc/math/Utils.h"

#include <cmath>
#include <numeric>

double DistanceBetweenPoints(const std::array<double, 3> first_point,
    const std::array<double, 3> second_point) {
  double inner_product = std::inner_product(
    first_point.begin(), first_point.end(), second_point.begin(), 0.0);
  return std::sqrt(inner_product);  
}