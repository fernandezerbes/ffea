#include "../../inc/math/utils.h"

#include <cmath>
#include <numeric>

namespace ffea {

namespace utilities {

double DistanceBetweenPoints(const std::array<double, 3>& first_point,
                             const std::array<double, 3>& second_point) {
  std::array<double, 3> distance_vector = {second_point[0] - first_point[0],
                                           second_point[1] - first_point[1],
                                           second_point[2] - first_point[2]};

  double inner_product =
      std::inner_product(distance_vector.begin(), distance_vector.end(),
                         distance_vector.begin(), 0.0);
  return std::sqrt(inner_product);
}

}  // namespace utilities

}  // namespace ffea