#include "../../inc/model/boundary_condition.h"

#include <stdexcept>

namespace ffea {

NaturalBoundaryCondition::NaturalBoundaryCondition(const std::vector<Element> &elements,
                                                   VectorialFunction load,
                                                   VectorialFunction radiation)
    : PhysicsProcessor(elements), load_(load), radiation_(radiation) {}

void NaturalBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                       Vector<double> &system_rhs) const {
  for (auto &element : elements_) {
    auto number_of_dofs = element.number_of_dofs();
    Vector<double> element_rhs = Vector<double>::Zero(number_of_dofs);
    Matrix<double> element_stiffness;
    if (radiation_) {
      element_stiffness = Matrix<double>::Zero(number_of_dofs, number_of_dofs);
    }

    for (const auto &ip : element.integration_points()) {
      const auto &local_coords = ip.local_coords();
      const auto &N = element.EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
      const auto &global_coords = element.MapLocalToGlobal(N);
      const auto &load_vector = load_(global_coords);
      const auto &weight = ip.weight();
      const auto &differential = element.EvaluateDifferential(local_coords);
      const auto &number_of_nodes = element.number_of_nodes();

      AddLoadContribution(number_of_nodes, load_vector, N, weight, differential, element_rhs);

      if (radiation_) {
        const auto radiation_value = radiation_(global_coords)[0];
        element_stiffness.triangularView<Eigen::Upper>() +=
            N.transpose() * radiation_value * N * weight * differential;
      }
    }

    const auto &dof_tags = element.dof_tags();
    if (radiation_) {
      Scatter(dof_tags, element_stiffness, element_rhs, system_stiffness, system_rhs);
    } else {
      Scatter(dof_tags, element_rhs, system_rhs);
    }
  }
}

void NaturalBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                       CSRMatrix<double> &system_mass,
                                       Vector<double> &system_rhs) const {
  throw std::runtime_error(
      "NaturalBoundaryCondition::Process not implemented for transient problems");
}

void NaturalBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                       CSRMatrix<double> &system_mass,
                                       CSRMatrix<double> &system_damping,
                                       Vector<double> &system_rhs) const {
  throw std::runtime_error(
      "NaturalBoundaryCondition::Process not implemented for damped transient problems");
}

PenaltyEnforcementStrategy::PenaltyEnforcementStrategy(double penalty) : penalty_(penalty) {}

void PenaltyEnforcementStrategy::Enforce(
    CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs, VectorialFunction condition,
    const std::vector<Element> &elements,
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
      system_stiffness.coeffRef(i_dof_tag, i_dof_tag) += penalty_;
      system_rhs(i_dof_tag) += boundary_values[component] * penalty_;
    }
  }
}

EssentialBoundaryCondition::EssentialBoundaryCondition(
    const std::vector<Element> &elements, VectorialFunction condition,
    const std::unordered_set<size_t> &components_to_consider, const EnforcementStrategy &strategy)
    : PhysicsProcessor(elements),
      condition_(condition),
      directions_to_consider_(components_to_consider),
      strategy_(strategy) {}

void EssentialBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                         Vector<double> &system_rhs) const {
  strategy_.Enforce(system_stiffness, system_rhs, condition_, elements_, directions_to_consider_);
}

void EssentialBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                         CSRMatrix<double> &system_mass,
                                         Vector<double> &system_rhs) const {
  throw std::runtime_error(
      "EssentialBoundaryCondition::Process not implemented for transient problems");
}

void EssentialBoundaryCondition::Process(CSRMatrix<double> &system_stiffness,
                                         CSRMatrix<double> &system_mass,
                                         CSRMatrix<double> &system_damping,
                                         Vector<double> &system_rhs) const {
  throw std::runtime_error(
      "EssentialBoundaryCondition::Process not implemented for damped transient problems");
}

}  // namespace ffea
