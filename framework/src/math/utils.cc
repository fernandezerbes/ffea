#include "../../inc/math/utils.h"

#include <cmath>
#include <numeric>

namespace ffea::utilities {

double DistanceBetweenPoints(const std::array<double, 3>& first,
                             const std::array<double, 3>& second) {
  std::array<double, 3> distance_vector = {second[0] - first[0], second[1] - first[1],
                                           second[2] - first[2]};

  const double inner_product = std::inner_product(distance_vector.begin(), distance_vector.end(),
                                                  distance_vector.begin(), 0.0);
  return std::sqrt(inner_product);
}

}  // namespace ffea::utilities
