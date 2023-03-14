#ifndef FFEA_FRAMEWORK_MODEL_OPERATOR_H_
#define FFEA_FRAMEWORK_MODEL_OPERATOR_H_

#include <functional>

#include "../alias.h"

namespace ffea {

using DifferentialOperator = std::function<Matrix<double>(const Matrix<double> &)>;

const DifferentialOperator gradient_operator =
    [](const Matrix<double> &dN_global) -> Matrix<double> { return dN_global; };

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_OPERATOR_H_
