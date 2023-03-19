#ifndef FFEA_FRAMEWORK_INC_ALIAS_H_
#define FFEA_FRAMEWORK_INC_ALIAS_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <functional>
#include <vector>

#include "./geometry/coordinates.h"

namespace ffea {

template <typename ScalarType>
using CSRMatrix = Eigen::SparseMatrix<ScalarType, Eigen::RowMajor>;

template <typename ScalarType>
using MatrixEntry = Eigen::Triplet<ScalarType>;

template <typename ScalarType>
using MatrixEntries = std::vector<MatrixEntry<ScalarType>>;

template <typename ScalarType>
using Matrix = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;

template <typename ScalarType>
using Vector = Eigen::Vector<ScalarType, Eigen::Dynamic>;

template <typename T>
using SpatioTemporalFunction = std::function<T(const Coordinates &, double)>;

using ValuesProcessor = std::function<Matrix<double>(const Vector<double> &, const Coordinates &,
                                                     const Matrix<double> &)>;

using NodalValues = std::vector<double>;

using NodalValuesGroup = std::vector<NodalValues>;

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_ALIAS_H_
