#ifndef FFEA_FRAMEWORK_MATH_UTILS_H_
#define FFEA_FRAMEWORK_MATH_UTILS_H_

#include <array>

namespace ffea {

namespace utilities {

double DistanceBetweenPoints(const std::array<double, 3>& first,
                             const std::array<double, 3>& second);

}  // namespace utilities

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MATH_UTILS_H_
