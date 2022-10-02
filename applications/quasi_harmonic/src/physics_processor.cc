#include "../inc/physics_processor.h"

#include "../../../framework/inc/model/operator.h"

namespace ffea {

QuasiHarmonicProcessor::QuasiHarmonicProcessor()
    : gradient_operator_(ffea::gradient_operator) {}

ElementSystem QuasiHarmonicProcessor::ProcessDomainElementSystem(
    const Element &element, const ConstitutiveModel &constitutive_model,
    ConditionFunction source) const {
  ElementSystem system{};
  const auto &number_of_dofs = element.GetNumberOfDofs();
  system.stiffness_matrix =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  if (source) {
    system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);
  }

  const auto &number_of_nodes = element.GetNumberOfNodes();
  const auto &number_of_components = element.GetNumberOfDofsPerNode();

  for (const auto &integration_point : element.integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &N = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = element.MapLocalToGlobal(N);
    const auto &dN_dLocal = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kFirst);
    const auto &jacobian = element.EvaluateJacobian(local_coords, dN_dLocal);
    const auto &dN_dGlobal = jacobian.inverse() * dN_dLocal;
    Eigen::MatrixXd B = gradient_operator_(dN_dGlobal);
    const auto &C = constitutive_model.Evaluate(global_coords);
    const auto &weight = integration_point.weight();
    const auto &differential = element.EvaluateDifferential(local_coords);

    *(system.stiffness_matrix) += B.transpose() * C * B * weight * differential;

    if (source) {
      const auto &load_vector = source(global_coords);
      AddLoadContributionToElementSystem(number_of_nodes, number_of_components,
                                         N, load_vector, weight, differential,
                                         system);
    }
  }

  return system;
}

ElementSystem QuasiHarmonicProcessor::ProcessBoundaryElementSystem(
    const Element &element, ConditionFunction load,
    ConditionFunction radiation) const {
  ElementSystem system{};
  const auto &number_of_dofs = element.GetNumberOfDofs();
  system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);

  if (radiation) {
    system.stiffness_matrix =
        Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  }

  const auto &number_of_nodes = element.GetNumberOfNodes();
  const auto &number_of_components = element.GetNumberOfDofsPerNode();

  for (const auto &integration_point : element.integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &N = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = element.MapLocalToGlobal(N);
    const auto &load_vector = load(global_coords);
    const auto &weight = integration_point.weight();
    const auto &differential = element.EvaluateDifferential(local_coords);
    AddLoadContributionToElementSystem(number_of_nodes, number_of_components, N,
                                       load_vector, weight, differential,
                                       system);
    if (radiation) {
      const auto &radiation_value = radiation(global_coords)[0];
      AddRadiationContributionToElementSystem(N, radiation_value, weight,
                                              differential, system);
    }
  }

  return system;
}

}  // namespace ffea
