#ifndef FFEA_FRAMEWORK_INC_MODEL_TYPES_H_
#define FFEA_FRAMEWORK_INC_MODEL_TYPES_H_

#include <Eigen/Dense>
#include <functional>
#include <vector>

#include "../geometry/coordinates.h"

namespace ffea {

using ConditionFunction =
    std::function<std::vector<double>(const Coordinates &)>;

using Integrand = std::function<Eigen::MatrixXd(const Eigen::MatrixXd &,
                                                const Eigen::MatrixXd &)>;

using ValuesProcessor = std::function<Eigen::MatrixXd(
    const Eigen::VectorXd &, const Coordinates &, const Eigen::MatrixXd &)>;

using NodalValues = std::vector<double>;

using NodalValuesGroup = std::vector<NodalValues>;

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_TYPES_H_