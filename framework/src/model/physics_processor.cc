#include "../../inc/model/physics_processor.h"

#include "../../inc/model/operator.h"

namespace ffea {

PhysicsProcessor::PhysicsProcessor(const std::vector<Element> &elements)
    : elements_(elements) {}

void PhysicsProcessor::AddContribution(Eigen::MatrixXd &global_stiffness,
                                       Eigen::VectorXd &global_rhs) const {
  for (auto &element : elements_) {
    const auto &element_system = ProcessElementSystem(element);
    Scatter(element, element_system, global_stiffness, global_rhs);
  }
}

void PhysicsProcessor::Scatter(const Element &element,
                               const ElementSystem &element_system,
                               Eigen::MatrixXd &global_stiffness,
                               Eigen::VectorXd &global_rhs) const {
  const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
  for (size_t node_index = 0; node_index < element.GetNumberOfNodes();
       node_index++) {
    size_t local_i_index = 0;
    for (const auto &global_i_index : dofs_map) {
      size_t local_j_index = 0;
      for (const auto &global_j_index : dofs_map) {
        if (element_system.stiffness_matrix) {
          global_stiffness(global_i_index, global_j_index) +=
              (*element_system.stiffness_matrix)(local_i_index, local_j_index);
        }
        local_j_index++;
      }
      if (element_system.rhs_vector) {
        global_rhs(global_i_index) +=
            (*element_system.rhs_vector)(local_i_index);
      }
      local_i_index++;
    }
  }
}

ElasticityDomainProcessor::ElasticityDomainProcessor(
    const std::vector<Element> &elements,
    const ConstitutiveModel &constitutive_model, ConditionFunction source,
    DifferentialOperator B_operator)
    : PhysicsProcessor(elements),
      constitutive_model_(constitutive_model),
      source_(source),
      B_operator_(B_operator) {}

ElementSystem ElasticityDomainProcessor::ProcessElementSystem(
    const Element &element) const {
  ElementSystem system{};
  const auto &number_of_dofs = element.GetNumberOfDofs();
  system.stiffness_matrix =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);

  for (const auto &integration_point : element.integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &differential = element.EvaluateDifferential(local_coords);
    const auto &dN_dLocal = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kFirst);
    const auto &jacobian = element.EvaluateJacobian(local_coords, dN_dLocal);
    const auto &dN_dGlobal = jacobian.inverse() * dN_dLocal;
    Eigen::MatrixXd B = B_operator_(dN_dGlobal);
    const auto &global_coords = element.MapLocalToGlobal(local_coords);
    const auto &C = constitutive_model_.Evaluate(global_coords);

    *(system.stiffness_matrix) +=
        B.transpose() * C * B * integration_point.weight() * differential;
  }

  return system;
}

ElasticityBoundaryProcessor::ElasticityBoundaryProcessor(
    const std::vector<Element> &elements, size_t dimensions,
    ConditionFunction load)
    : PhysicsProcessor(elements), dimensions_(dimensions), load_(load) {}

ElementSystem ElasticityBoundaryProcessor::ProcessElementSystem(
    const Element &element) const {
  ElementSystem system{};
  const auto &number_of_dofs = element.GetNumberOfDofs();
  Eigen::VectorXd rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);

  for (const auto &integration_point : element.integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &N = element.EvaluateShapeFunctions(
        local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = element.MapLocalToGlobal(N);
    const auto &load_vector = load_(global_coords);
    const auto &number_of_nodes = element.GetNumberOfNodes();
    const auto &number_of_dofs = dimensions_ * number_of_nodes;
    const auto &differential = element.EvaluateDifferential(local_coords);

    for (auto node_idx = 0; node_idx < number_of_nodes; node_idx++) {
      for (auto dimension_idx = 0; dimension_idx < dimensions_;
           dimension_idx++) {
        const auto &dof_idx = node_idx * dimensions_ + dimension_idx;
        rhs_vector(dof_idx) += N(0, node_idx) * load_vector[dimension_idx] *
                               integration_point.weight() * differential;
      }
    }
  }

  system.rhs_vector = rhs_vector;  // TODO Fix this, directly set the values
  return system;
}

}  // namespace ffea
