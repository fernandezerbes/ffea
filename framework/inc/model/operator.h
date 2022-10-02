#ifndef FFEA_FRAMEWORK_MODEL_OPERATOR_H_
#define FFEA_FRAMEWORK_MODEL_OPERATOR_H_

#include <eigen3/Eigen/Dense>
#include <functional>

namespace ffea {

using DifferentialOperator =
    std::function<Eigen::MatrixXd(const Eigen::MatrixXd &)>;

const DifferentialOperator linear_B_operator_2D =
    [](const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
  auto dimensions = 2;
  auto number_of_shape_functions = dN_dGlobal.cols();
  auto columns = number_of_shape_functions * dimensions;
  auto rows = 3;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(rows, columns);

  for (auto N_idx = 0; N_idx < number_of_shape_functions; N_idx++) {
    auto first_dof_idx = dimensions * N_idx;
    auto second_dof_idx = first_dof_idx + 1;
    B(0, first_dof_idx) = dN_dGlobal(0, N_idx);
    B(1, second_dof_idx) = dN_dGlobal(1, N_idx);
    B(2, first_dof_idx) = dN_dGlobal(1, N_idx);
    B(2, second_dof_idx) = dN_dGlobal(0, N_idx);
  }

  return B;
};

const DifferentialOperator linear_B_operator_3D =
    [](const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
  auto dimensions = 3;
  auto number_of_shape_functions = dN_dGlobal.cols();
  auto columns = number_of_shape_functions * dimensions;
  auto rows = 6;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(rows, columns);

  for (auto N_idx = 0; N_idx < number_of_shape_functions; N_idx++) {
    auto first_dof_idx = dimensions * N_idx;
    auto second_dof_idx = first_dof_idx + 1;
    auto third_dof_idx = second_dof_idx + 1;
    B(0, first_dof_idx) = dN_dGlobal(0, N_idx);
    B(1, second_dof_idx) = dN_dGlobal(1, N_idx);
    B(2, third_dof_idx) = dN_dGlobal(2, N_idx);
    B(3, second_dof_idx) = dN_dGlobal(2, N_idx);
    B(3, third_dof_idx) = dN_dGlobal(1, N_idx);
    B(4, first_dof_idx) = dN_dGlobal(2, N_idx);
    B(4, third_dof_idx) = dN_dGlobal(0, N_idx);
    B(5, first_dof_idx) = dN_dGlobal(1, N_idx);
    B(5, second_dof_idx) = dN_dGlobal(0, N_idx);
  }

  return B;
};

const DifferentialOperator gradient_operator =
    [](const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
  return dN_dGlobal;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_OPERATOR_H_
