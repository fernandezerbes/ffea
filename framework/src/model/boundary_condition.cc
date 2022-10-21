#include "../../inc/model/boundary_condition.h"

namespace ffea {

BoundaryCondition::BoundaryCondition(const std::vector<Element> &elements)
    : elements_(elements) {}

NaturalBoundaryCondition::NaturalBoundaryCondition(
    const std::vector<Element> &elements, ConditionFunction load,
    ConditionFunction radiation)
    : BoundaryCondition(elements), load_(load), radiation_(radiation) {}

void NaturalBoundaryCondition::Enforce(Eigen::MatrixXd &global_stiffness,
                                       Eigen::VectorXd &global_rhs) const {
  for (auto &element : elements_) {
    element.ProcessOverBoundary(load_, radiation_, global_stiffness,
                                global_rhs);
  }
}

// void DirectEnforcementStrategy::Enforce(
//     Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
//     ConditionFunction condition,
//     const std::vector<Element> &elements,
//     const std::unordered_set<size_t> &components_to_consider) const {
//   // TODO Take care of the case where more than one bc is applied to the
//   same boundary for (auto &element : elements) {
//     const auto &dof_tags = element.dof_tags();
//     for (size_t i_dof_idx = 0; i_dof_idx < dof_tags.size();
//     i_dof_idx++) {
//       size_t component = i_dof_idx %
//       element.dofs_per_node(); if
//       (!components_to_consider.contains(component)) {
//         continue;
//       }
//       size_t node_idx = i_dof_idx /
//       element.dofs_per_node(); const auto &node_coords =
//       element.node_coords(node_idx); auto boundary_values =
//       condition(node_coords); const auto &i_dof_tag =
//       dof_tags[i_dof_idx]; global_stiffness.coeffRef(i_dof_tag,
//       i_dof_tag) = 1.0; global_rhs(i_dof_tag) =
//       boundary_values[component] * penalty_;
//     }
//   }
// }

PenaltyEnforcementStrategy::PenaltyEnforcementStrategy(double penalty)
    : penalty_(penalty) {}

void PenaltyEnforcementStrategy::Enforce(
    Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
    ConditionFunction condition, const std::vector<Element> &elements,
    const std::unordered_set<size_t> &components_to_consider) const {
  for (auto &element : elements) {
    const auto &dof_tags = element.dof_tags();
    for (size_t i_dof_idx = 0; i_dof_idx < dof_tags.size(); i_dof_idx++) {
      size_t component = i_dof_idx % element.dofs_per_node();
      if (!components_to_consider.contains(component)) {
        continue;
      }
      size_t node_idx = i_dof_idx / element.dofs_per_node();
      const auto &coords = element.node_coords(node_idx);
      auto boundary_values = condition(coords);
      const auto &i_dof_tag = dof_tags[i_dof_idx];
      global_stiffness.coeffRef(i_dof_tag, i_dof_tag) += penalty_;
      global_rhs(i_dof_tag) += boundary_values[component] * penalty_;
    }
  }
}

EssentialBoundaryCondition::EssentialBoundaryCondition(
    const std::vector<Element> &elements, ConditionFunction condition,
    const std::unordered_set<size_t> &components_to_consider,
    const EnforcementStrategy &strategy)
    : BoundaryCondition(elements),
      condition_(condition),
      directions_to_consider_(components_to_consider),
      strategy_(strategy) {}

void EssentialBoundaryCondition::Enforce(Eigen::MatrixXd &global_stiffness,
                                         Eigen::VectorXd &global_rhs) const {
  strategy_.Enforce(global_stiffness, global_rhs, condition_, elements_,
                    directions_to_consider_);
}

}  // namespace ffea
