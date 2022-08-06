#include "../../inc/processor/operator.h"

namespace ffea {

DifferentialOperator::DifferentialOperator(size_t dofs_per_node)
    : dofs_per_node_(dofs_per_node) {}

StrainDisplacementOperator2D::StrainDisplacementOperator2D()
    : DifferentialOperator(2) {}

const Eigen::MatrixXd StrainDisplacementOperator2D::Compute(
    const Eigen::MatrixXd &shape_function_derivatives) const {
  size_t number_of_nodes = shape_function_derivatives.cols();

  Eigen::MatrixXd differential_operator =
      Eigen::MatrixXd::Zero(3, number_of_nodes * dofs_per_node_);

  for (size_t node_index = 0; node_index < number_of_nodes; node_index++) {
    auto first_dof_index = dofs_per_node_ * node_index;
    auto second_dof_index = first_dof_index + 1;
    differential_operator(0, first_dof_index) =
        shape_function_derivatives(0, node_index);
    differential_operator(1, second_dof_index) =
        shape_function_derivatives(1, node_index);
    differential_operator(2, first_dof_index) =
        shape_function_derivatives(1, node_index);
    differential_operator(2, second_dof_index) =
        shape_function_derivatives(0, node_index);
  }

  return differential_operator;
}

}  // namespace ffea
