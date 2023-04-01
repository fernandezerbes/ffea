#include "../../inc/model/boundary_condition.h"

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace ffea {

EssentialBoundaryCondition::EssentialBoundaryCondition(
    std::vector<Element> &elements, SpatioTemporalFunction<std::vector<double>> condition,
    const std::unordered_set<size_t> &components_to_consider, EnforcementStrategy &strategy)
    : elements_(elements),
      condition_(std::move(condition)),
      directions_to_consider_(components_to_consider),
      strategy_(strategy) {}

void EssentialBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                         Vector<double> &system_rhs, Time t) {
  strategy_.Enforce(system_stiffness, system_rhs, condition_, elements_, directions_to_consider_,
                    t);
}

void DirectEnforcementStrategy::Enforce(CSRMatrix<double> &system_stiffness,
                                        Vector<double> &system_rhs,
                                        SpatioTemporalFunction<std::vector<double>> condition,
                                        std::vector<Element> &elements,
                                        const std::unordered_set<size_t> &components_to_consider,
                                        Time t) {
  // We initialize free dofs with all dofs, and later we remove the constrained dofs
  std::unordered_set<size_t> free_dof_tags{};
  free_dof_tags.reserve(system_rhs.rows());
  for (size_t dof_tag = 0; dof_tag < system_rhs.rows(); dof_tag++) {
    free_dof_tags.insert(free_dof_tags.end(), dof_tag);
  }

  std::unordered_map<size_t, double> constrained_dof_tag_to_values_map{};
  constrained_dof_tag_to_values_map.reserve(system_rhs.rows());

  // Set all constrained values on the rhs and cache free and constrained dofs
  for (auto &element : elements) {
    const auto &dof_tags = element.dof_tags();
    for (size_t i_dof_idx = 0; i_dof_idx < dof_tags.size(); i_dof_idx++) {
      size_t component = i_dof_idx % element.dofs_per_node();
      if (!components_to_consider.contains(component)) {
        continue;
      }
      size_t node_idx = i_dof_idx / element.dofs_per_node();
      const auto &i_dof_tag = dof_tags[i_dof_idx];
      if (constrained_dof_tag_to_values_map.contains(i_dof_tag)) {
        continue;
      }
      const auto &coords = element.node_coords(node_idx);
      auto boundary_values = condition(coords, t);
      constrained_dof_tag_to_values_map.insert({i_dof_tag, boundary_values[component]});
      free_dof_tags.erase(i_dof_tag);

      system_rhs(i_dof_tag) = boundary_values[component];
    }
  }

  // Trivialize constrained rows (loop over *constrained* rows and set to zero *all* columns)
  for (auto [constrained_dof_tag, constrained_value] : constrained_dof_tag_to_values_map) {
    for (CSRMatrix<double>::InnerIterator it(system_stiffness, constrained_dof_tag); it; ++it) {
      if (it.row() == it.col()) {
        it.valueRef() = 1.0;
      } else {
        it.valueRef() = 0.0;
      }
    }
  }

  // Modify the rhs with the influence of stiffness on constrained dofd (loop over *free* rows and
  // pass to the rhs the contribution of *constrained* columns)
  for (auto free_dof_tag : free_dof_tags) {
    for (CSRMatrix<double>::InnerIterator it(system_stiffness, free_dof_tag); it; ++it) {
      if (constrained_dof_tag_to_values_map.contains(it.col())) {
        auto value = it.value() * constrained_dof_tag_to_values_map.at(it.col());
        system_rhs(free_dof_tag) -= value;
      }
    }
  }

  // Trivialize constrained columns (loop over *all* rows and set to zero *constrained* columns)
  for (int row = 0; row < system_stiffness.outerSize(); row++) {
    for (CSRMatrix<double>::InnerIterator it(system_stiffness, row); it; ++it) {
      if (!constrained_dof_tag_to_values_map.contains(it.col())) {
        continue;
      }

      if (it.row() == it.col()) {
        it.valueRef() = 1.0;
      } else {
        it.valueRef() = 0.0;
      }
    }
  }
}

PenaltyEnforcementStrategy::PenaltyEnforcementStrategy(double penalty) : penalty_(penalty) {}

void PenaltyEnforcementStrategy::Enforce(CSRMatrix<double> &system_stiffness,
                                         Vector<double> &system_rhs,
                                         SpatioTemporalFunction<std::vector<double>> condition,
                                         std::vector<Element> &elements,
                                         const std::unordered_set<size_t> &components_to_consider,
                                         Time t) {
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
