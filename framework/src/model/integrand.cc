#include "../../inc/model/integrand.h"

namespace ffea {

Integrand::Integrand(size_t physical_dimension)
    : physical_dimension_(physical_dimension) {}

ElasticityBaseIntegrand::ElasticityBaseIntegrand(
    size_t physical_dimension, const ConstitutiveModel &constitutive_model,
    ConditionFunction source)
    : Integrand(physical_dimension),
      constitutive_model_(constitutive_model),
      source_(source) {}

const Eigen::MatrixXd ElasticityBaseIntegrand::Compute(
    const GeometricEntity &geometric_entity,
    const Coordinates &local_coordinates) const {
  const auto &local_shape_functions_derivatives =
      geometric_entity.EvaluateShapeFunctions(local_coordinates,
                                              ffea::DerivativeOrder::kFirst);
  const auto &jacobian = geometric_entity.EvaluateJacobian(
      local_coordinates, local_shape_functions_derivatives);
  const auto &global_shape_functions_derivatives =
      jacobian.inverse() * local_shape_functions_derivatives;
  Eigen::MatrixXd operator_matrix =
      GetStrainDisplacementOperator(global_shape_functions_derivatives);
  const auto &global_coordinates =
      geometric_entity.MapLocalToGlobal(local_coordinates);
  const auto &constitutive_matrix =
      constitutive_model_.Evaluate(global_coordinates);
  return operator_matrix.transpose() * constitutive_matrix * operator_matrix;
}

Elasticity2DIntegrand::Elasticity2DIntegrand(
    const ConstitutiveModel &constitutive_model, ConditionFunction source)
    : ElasticityBaseIntegrand(2, constitutive_model, source) {}

const Eigen::MatrixXd Elasticity2DIntegrand::GetStrainDisplacementOperator(
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

Elasticity3DIntegrand::Elasticity3DIntegrand(
    const ConstitutiveModel &constitutive_model, ConditionFunction source)
    : ElasticityBaseIntegrand(3, constitutive_model, source) {}

const Eigen::MatrixXd Elasticity3DIntegrand::GetStrainDisplacementOperator(
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
