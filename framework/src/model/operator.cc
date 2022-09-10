#include "../../inc/model/operator.h"

namespace ffea {

DifferentialOperator::DifferentialOperator(size_t physical_dimension)
    : physical_dimension_(physical_dimension) {}

StrainDisplacementOperator2D::StrainDisplacementOperator2D()
    : DifferentialOperator(2) {}

const Eigen::MatrixXd StrainDisplacementOperator2D::Compute(
    const Eigen::MatrixXd &shape_function_derivatives) const {
  size_t number_of_shape_functions = shape_function_derivatives.cols();

  Eigen::MatrixXd differential_operator =
      Eigen::MatrixXd::Zero(3, number_of_shape_functions * physical_dimension_);

  for (size_t shape_function_index = 0;
       shape_function_index < number_of_shape_functions;
       shape_function_index++) {
    auto first_dof_index = physical_dimension_ * shape_function_index;
    auto second_dof_index = first_dof_index + 1;
    differential_operator(0, first_dof_index) =
        shape_function_derivatives(0, shape_function_index);
    differential_operator(1, second_dof_index) =
        shape_function_derivatives(1, shape_function_index);
    differential_operator(2, first_dof_index) =
        shape_function_derivatives(1, shape_function_index);
    differential_operator(2, second_dof_index) =
        shape_function_derivatives(0, shape_function_index);
  }

  return differential_operator;
}

StrainDisplacementOperator3D::StrainDisplacementOperator3D()
    : DifferentialOperator(3) {}

const Eigen::MatrixXd StrainDisplacementOperator3D::Compute(
    const Eigen::MatrixXd &shape_function_derivatives) const {
  size_t number_of_shape_functions = shape_function_derivatives.cols();

  Eigen::MatrixXd differential_operator =
      Eigen::MatrixXd::Zero(6, number_of_shape_functions * physical_dimension_);

  for (size_t shape_function_index = 0;
       shape_function_index < number_of_shape_functions;
       shape_function_index++) {
    auto first_dof_index = physical_dimension_ * shape_function_index;
    auto second_dof_index = first_dof_index + 1;
    auto third_dof_index = second_dof_index + 1;

    differential_operator(0, first_dof_index) =
        shape_function_derivatives(0, shape_function_index);

    differential_operator(1, second_dof_index) =
        shape_function_derivatives(1, shape_function_index);

    differential_operator(2, third_dof_index) =
        shape_function_derivatives(2, shape_function_index);

    differential_operator(3, second_dof_index) =
        shape_function_derivatives(2, shape_function_index);
    differential_operator(3, third_dof_index) =
        shape_function_derivatives(1, shape_function_index);

    differential_operator(4, first_dof_index) =
        shape_function_derivatives(2, shape_function_index);
    differential_operator(4, third_dof_index) =
        shape_function_derivatives(0, shape_function_index);

    differential_operator(5, first_dof_index) =
        shape_function_derivatives(1, shape_function_index);
    differential_operator(5, second_dof_index) =
        shape_function_derivatives(0, shape_function_index);
  }

  return differential_operator;
}

}  // namespace ffea
