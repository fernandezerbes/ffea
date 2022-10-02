#include "../../inc/model/physics_processor.h"

#include "../../inc/model/operator.h"

namespace ffea {

void PhysicsProcessor::AddDomainContribution(
    const std::vector<Element> &elements,
    const ConstitutiveModel &constitutive_model, ConditionFunction source,
    Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs) const {
  for (auto &element : elements) {
    const auto &element_system =
        ProcessDomainElementSystem(element, constitutive_model, source);
    Scatter(element, element_system, global_stiffness, global_rhs);
  }
}

void PhysicsProcessor::AddBoundaryContribution(
    const std::vector<Element> &elements, ConditionFunction load,
    Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs) const {
  for (auto &element : elements) {
    const auto &element_system = ProcessBoundaryElementSystem(element, load);
    Scatter(element, element_system, global_stiffness, global_rhs);
  }
}

void PhysicsProcessor::Scatter(const Element &element,
                               const ElementSystem &element_system,
                               Eigen::MatrixXd &global_stiffness,
                               Eigen::VectorXd &global_rhs) const {
  const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
  for (size_t node_idx = 0; node_idx < element.GetNumberOfNodes(); node_idx++) {
    size_t local_i_idx = 0;
    for (const auto &global_i_idx : dofs_map) {
      size_t local_j_idx = 0;
      for (const auto &global_j_idx : dofs_map) {
        if (element_system.stiffness_matrix) {
          global_stiffness(global_i_idx, global_j_idx) +=
              (*element_system.stiffness_matrix)(local_i_idx, local_j_idx);
        }
        local_j_idx++;
      }
      if (element_system.rhs_vector) {
        global_rhs(global_i_idx) += (*element_system.rhs_vector)(local_i_idx);
      }
      local_i_idx++;
    }
  }
}

ElasticityProcessor::ElasticityProcessor(DifferentialOperator B_operator)
    : B_operator_(B_operator) {}

ElementSystem ElasticityProcessor::ProcessDomainElementSystem(
    const Element &element, const ConstitutiveModel &constitutive_model,
    ConditionFunction source) const {
  ElementSystem system{};
  const auto &number_of_dofs = element.GetNumberOfDofs();
  system.stiffness_matrix =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  // if (source) {
  //   system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);
  // }

  for (const auto &integration_point : element.integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &dN_dLocal = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kFirst);
    const auto &jacobian = element.EvaluateJacobian(local_coords, dN_dLocal);
    const auto &dN_dGlobal = jacobian.inverse() * dN_dLocal;
    Eigen::MatrixXd B = B_operator_(dN_dGlobal);
    const auto &global_coords = element.MapLocalToGlobal(local_coords);
    const auto &C = constitutive_model.Evaluate(global_coords);
    const auto &differential = element.EvaluateDifferential(local_coords);

    *system.stiffness_matrix +=
        B.transpose() * C * B * integration_point.weight() * differential;

    // if (source) {
    //   system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);
    // }
  }

  return system;
}

ElementSystem ElasticityProcessor::ProcessBoundaryElementSystem(
    const Element &element, ConditionFunction load) const {
  ElementSystem system{};
  const auto &number_of_dofs = element.GetNumberOfDofs();
  system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);

  const auto &number_of_nodes = element.GetNumberOfNodes();
  const auto &number_of_components = element.GetNumberOfDofsPerNode();

  for (const auto &integration_point : element.integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &N = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = element.MapLocalToGlobal(N);
    const auto &load_vector = load(global_coords);
    const auto &differential = element.EvaluateDifferential(local_coords);

    for (auto node_idx = 0; node_idx < number_of_nodes; node_idx++) {
      for (auto component_idx = 0; component_idx < number_of_components;
           component_idx++) {
        const auto &dof_idx = node_idx * number_of_components + component_idx;
        (*system.rhs_vector)(dof_idx) +=
            N(0, node_idx) * load_vector[component_idx] *
            integration_point.weight() * differential;
      }
    }
  }

  return system;
}

}  // namespace ffea
