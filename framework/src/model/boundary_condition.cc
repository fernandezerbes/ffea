#include "../../inc/model/boundary_condition.h"

namespace ffea {

BoundaryCondition::BoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function)
    : boundary_elements_(boundary_elements),
      boundary_function_(boundary_function) {}

BoundaryCondition::~BoundaryCondition() {}

NeumannBoundaryCondition::NeumannBoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function)
    : BoundaryCondition(boundary_elements, boundary_function) {}

void NeumannBoundaryCondition::Enforce(Eigen::MatrixXd &global_stiffness,
                                       Eigen::VectorXd &global_rhs) const {
  for (auto &element : boundary_elements_) {
    const auto &element_rhs = element.ComputeRhs(boundary_function_);
    const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
    // Scatter coefficients
    for (size_t local_i_index = 0; local_i_index < dofs_map.size();
         local_i_index++) {
      size_t global_i_index = dofs_map[local_i_index];
      global_rhs(global_i_index) += element_rhs(local_i_index);
    }
  }
}

// void DirectEnforcementStrategy::Enforce(
//     Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
//     ConditionFunction boundary_function,
//     const std::vector<Element> &boundary_elements,
//     const std::unordered_set<size_t> &directions_to_consider) const {
//   // TODO Take care of the case where more than one bc is applied to the
//   same boundary for (auto &element : boundary_elements) {
//     const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
//     for (size_t local_i_index = 0; local_i_index < dofs_map.size();
//     local_i_index++) {
//       size_t dof_direction = local_i_index %
//       element.GetNumberOfDofsPerNode(); if
//       (!directions_to_consider.contains(dof_direction)) {
//         continue;
//       }
//       size_t node_index = local_i_index /
//       element.GetNumberOfDofsPerNode(); const auto &node_coordinates =
//       element.GetCoordinatesOfNode(node_index); auto boundary_values =
//       boundary_function(node_coordinates); const auto &global_i_index =
//       dofs_map[local_i_index]; global_stiffness.coeffRef(global_i_index,
//       global_i_index) = 1.0; global_rhs(global_i_index) =
//       boundary_values[dof_direction] * penalty_;
//     }
//   }
// }

PenaltyEnforcementStrategy::PenaltyEnforcementStrategy(double penalty)
    : penalty_(penalty) {}

void PenaltyEnforcementStrategy::Enforce(
    Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
    ConditionFunction boundary_function,
    const std::vector<Element> &boundary_elements,
    const std::unordered_set<size_t> &directions_to_consider) const {
  for (auto &element : boundary_elements) {
    const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
    for (size_t local_i_index = 0; local_i_index < dofs_map.size();
         local_i_index++) {
      size_t dof_direction = local_i_index % element.GetNumberOfDofsPerNode();
      if (!directions_to_consider.contains(dof_direction)) {
        continue;
      }
      size_t node_index = local_i_index / element.GetNumberOfDofsPerNode();
      const auto &node_coordinates = element.GetCoordinatesOfNode(node_index);
      auto boundary_values = boundary_function(node_coordinates);
      const auto &global_i_index = dofs_map[local_i_index];
      global_stiffness.coeffRef(global_i_index, global_i_index) += penalty_;
      global_rhs(global_i_index) += boundary_values[dof_direction] * penalty_;
    }
  }
}

DirichletBoundaryCondition::DirichletBoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function,
    const std::unordered_set<size_t> &directions_to_consider,
    const EnforcementStrategy &enforcement_strategy)
    : BoundaryCondition(boundary_elements, boundary_function),
      directions_to_consider_(directions_to_consider),
      enforcement_strategy_(enforcement_strategy) {}

void DirichletBoundaryCondition::Enforce(Eigen::MatrixXd &global_stiffness,
                                         Eigen::VectorXd &global_rhs) const {
  enforcement_strategy_.Enforce(global_stiffness, global_rhs,
                                boundary_function_, boundary_elements_,
                                directions_to_consider_);
}

}  // namespace ffea