#ifndef FFEA_FRAMEWORK_MATH_UTILS_H_
#define FFEA_FRAMEWORK_MATH_UTILS_H_

#include <vector>

namespace ffea {

namespace utilities {

double DistanceBetweenPoints(const std::vector<double>& first_point,
                             const std::vector<double>& second_point);

}  // namespace utilities

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MATH_UTILS_H_
