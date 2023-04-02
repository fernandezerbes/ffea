#ifndef FFEA_FRAMEWORK_MATH_UTILS_H_
#define FFEA_FRAMEWORK_MATH_UTILS_H_

#include <array>

namespace ffea::utilities {

double DistanceBetweenPoints(const std::array<double, 3> &first,
                             const std::array<double, 3> &second);

}  // namespace ffea::utilities

#endif  // FFEA_FRAMEWORK_MATH_UTILS_H_
