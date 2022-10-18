#include "../../inc/model/boundary_condition.h"

namespace ffea {

BoundaryCondition::BoundaryCondition(
    const std::vector<Element> &boundary_elements)
    : boundary_elements_(boundary_elements) {}

NaturalBoundaryCondition::NaturalBoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_load, ConditionFunction radiation)
    : BoundaryCondition(boundary_elements),
      boundary_load_(boundary_load),
      radiation_(radiation) {}

void NaturalBoundaryCondition::Enforce(Eigen::MatrixXd &global_stiffness,
                                       Eigen::VectorXd &global_rhs) const {
  for (auto &element : boundary_elements_) {
    element.ProcessOverBoundary(boundary_load_, radiation_, global_stiffness,
                                global_rhs);
  }
}

// void DirectEnforcementStrategy::Enforce(
//     Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
//     ConditionFunction boundary_function,
//     const std::vector<Element> &boundary_elements,
//     const std::unordered_set<size_t> &directions_to_consider) const {
//   // TODO Take care of the case where more than one bc is applied to the
//   same boundary for (auto &element : boundary_elements) {
//     const auto &dofs_map = element.GetDofTags();
//     for (size_t local_i_idx = 0; local_i_idx < dofs_map.size();
//     local_i_idx++) {
//       size_t dof_direction = local_i_idx %
//       element.GetNumberOfDofsPerNode(); if
//       (!directions_to_consider.contains(dof_direction)) {
//         continue;
//       }
//       size_t node_idx = local_i_idx /
//       element.GetNumberOfDofsPerNode(); const auto &node_coords =
//       element.GetCoordinatesOfNode(node_idx); auto boundary_values =
//       boundary_function(node_coords); const auto &global_i_idx =
//       dofs_map[local_i_idx]; global_stiffness.coeffRef(global_i_idx,
//       global_i_idx) = 1.0; global_rhs(global_i_idx) =
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
    const auto &dofs_map = element.GetDofTags();
    for (size_t local_i_idx = 0; local_i_idx < dofs_map.size(); local_i_idx++) {
      size_t dof_direction = local_i_idx % element.GetNumberOfDofsPerNode();
      if (!directions_to_consider.contains(dof_direction)) {
        continue;
      }
      size_t node_idx = local_i_idx / element.GetNumberOfDofsPerNode();
      const auto &node_coords = element.GetCoordinatesOfNode(node_idx);
      auto boundary_values = boundary_function(node_coords);
      const auto &global_i_idx = dofs_map[local_i_idx];
      global_stiffness.coeffRef(global_i_idx, global_i_idx) += penalty_;
      global_rhs(global_i_idx) += boundary_values[dof_direction] * penalty_;
    }
  }
}

EssentialBoundaryCondition::EssentialBoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function,
    const std::unordered_set<size_t> &directions_to_consider,
    const EnforcementStrategy &enforcement_strategy)
    : BoundaryCondition(boundary_elements),
      boundary_function_(boundary_function),
      directions_to_consider_(directions_to_consider),
      enforcement_strategy_(enforcement_strategy) {}

void EssentialBoundaryCondition::Enforce(Eigen::MatrixXd &global_stiffness,
                                         Eigen::VectorXd &global_rhs) const {
  enforcement_strategy_.Enforce(global_stiffness, global_rhs,
                                boundary_function_, boundary_elements_,
                                directions_to_consider_);
}

}  // namespace ffea