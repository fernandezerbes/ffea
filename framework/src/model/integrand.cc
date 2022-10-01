#include "../../inc/model/integrand.h"
#include "../../inc/model/operator.h"

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
  return ffea::linear_B_operator_2D(shape_function_derivatives);
}

Elasticity3DIntegrand::Elasticity3DIntegrand(
    const ConstitutiveModel &constitutive_model, ConditionFunction source)
    : ElasticityBaseIntegrand(3, constitutive_model, source) {}

const Eigen::MatrixXd Elasticity3DIntegrand::GetStrainDisplacementOperator(
    const Eigen::MatrixXd &shape_function_derivatives) const {
  return ffea::linear_B_operator_3D(shape_function_derivatives);
}

}  // namespace ffea
