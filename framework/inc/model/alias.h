#ifndef FFEA_FRAMEWORK_INC_MODEL_TYPES_H_
#define FFEA_FRAMEWORK_INC_MODEL_TYPES_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
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

template<typename ScalarType>
using CSRMatrix = Eigen::SparseMatrix<ScalarType, Eigen::RowMajor>;

template<typename ScalarType>
using MatrixEntry = Eigen::Triplet<ScalarType>;

template<typename ScalarType>
using MatrixEntries = std::vector<MatrixEntry<ScalarType>>;

template<typename ScalarType>
using Matrix = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;

template<typename ScalarType>
using Vector = Eigen::Vector<ScalarType, Eigen::Dynamic>;


}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_TYPES_H_
