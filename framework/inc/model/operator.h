#ifndef FFEA_FRAMEWORK_MODEL_OPERATOR_H_
#define FFEA_FRAMEWORK_MODEL_OPERATOR_H_

#include <functional>

#include "../alias.h"

namespace ffea {

using DifferentialOperator = std::function<Matrix<double>(const Matrix<double> &)>;

const DifferentialOperator linear_B_operator_2D =
    [](const Matrix<double> &dN_global) -> Matrix<double> {
  auto dim = 2;
  auto number_of_shape_functions = dN_global.cols();
  auto columns = number_of_shape_functions * dim;
  auto rows = 3;
  Matrix<double> B = Matrix<double>::Zero(rows, columns);

  for (auto N_idx = 0; N_idx < number_of_shape_functions; N_idx++) {
    auto first_dof_idx = dim * N_idx;
    auto second_dof_idx = first_dof_idx + 1;
    B(0, first_dof_idx) = dN_global(0, N_idx);
    B(1, second_dof_idx) = dN_global(1, N_idx);
    B(2, first_dof_idx) = dN_global(1, N_idx);
    B(2, second_dof_idx) = dN_global(0, N_idx);
  }

  return B;
};

const DifferentialOperator linear_B_operator_3D =
    [](const Matrix<double> &dN_global) -> Matrix<double> {
  auto dim = 3;
  auto number_of_shape_functions = dN_global.cols();
  auto columns = number_of_shape_functions * dim;
  auto rows = 6;
  Matrix<double> B = Matrix<double>::Zero(rows, columns);

  for (auto N_idx = 0; N_idx < number_of_shape_functions; N_idx++) {
    auto first_dof_idx = dim * N_idx;
    auto second_dof_idx = first_dof_idx + 1;
    auto third_dof_idx = second_dof_idx + 1;
    B(0, first_dof_idx) = dN_global(0, N_idx);
    B(1, second_dof_idx) = dN_global(1, N_idx);
    B(2, third_dof_idx) = dN_global(2, N_idx);
    B(3, second_dof_idx) = dN_global(2, N_idx);
    B(3, third_dof_idx) = dN_global(1, N_idx);
    B(4, first_dof_idx) = dN_global(2, N_idx);
    B(4, third_dof_idx) = dN_global(0, N_idx);
    B(5, first_dof_idx) = dN_global(1, N_idx);
    B(5, second_dof_idx) = dN_global(0, N_idx);
  }

  return B;
};

const DifferentialOperator gradient_operator =
    [](const Matrix<double> &dN_global) -> Matrix<double> { return dN_global; };

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_OPERATOR_H_
