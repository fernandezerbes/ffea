#include "../../inc/model/boundary_condition.h"

#include <stdexcept>

namespace ffea {

EssentialBoundaryCondition::EssentialBoundaryCondition(
    std::vector<Element> &elements, SpatioTemporalFunction<std::vector<double>> condition,
    const std::unordered_set<size_t> &components_to_consider, EnforcementStrategy &strategy)
    : elements_(elements),
      condition_(condition),
      directions_to_consider_(components_to_consider),
      strategy_(strategy) {}

void EssentialBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                         Vector<double> &system_rhs, double t) {
  strategy_.Enforce(system_stiffness, system_rhs, condition_, elements_, directions_to_consider_,
                    t);
}

PenaltyEnforcementStrategy::PenaltyEnforcementStrategy(double penalty) : penalty_(penalty) {}

void PenaltyEnforcementStrategy::Enforce(CSRMatrix<double> &system_stiffness,
                                         Vector<double> &system_rhs,
                                         SpatioTemporalFunction<std::vector<double>> condition,
                                         std::vector<Element> &elements,
                                         const std::unordered_set<size_t> &components_to_consider,
                                         double t) {
  for (auto &element : elements) {
    const auto &dof_tags = element.dof_tags();
    for (size_t i_dof_idx = 0; i_dof_idx < dof_tags.size(); i_dof_idx++) {
      size_t component = i_dof_idx % element.dofs_per_node();
      if (!components_to_consider.contains(component)) {
        continue;
      }
      size_t node_idx = i_dof_idx / element.dofs_per_node();
      const auto &coords = element.node_coords(node_idx);
      auto boundary_values = condition(coords, t);
      const auto &i_dof_tag = dof_tags[i_dof_idx];
      system_stiffness.coeffRef(i_dof_tag, i_dof_tag) += penalty_;
      system_rhs(i_dof_tag) += boundary_values[component] * penalty_;
    }
  }
}

}  // namespace ffea
