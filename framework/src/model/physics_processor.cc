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
    ConditionFunction radiation, Eigen::MatrixXd &global_stiffness,
    Eigen::VectorXd &global_rhs) const {
  for (auto &element : elements) {
    const auto &element_system =
        ProcessBoundaryElementSystem(element, load, radiation);
    Scatter(element, element_system, global_stiffness, global_rhs);
  }
}

void PhysicsProcessor::AddLoadContributionToElementSystem(
    size_t number_of_nodes, size_t number_of_components,
    const Eigen::MatrixXd &N, const std::vector<double> &load_vector,
    double weight, double differential, ElementSystem &system) const {
  for (auto node_idx = 0; node_idx < number_of_nodes; node_idx++) {
    for (auto component_idx = 0; component_idx < number_of_components;
         component_idx++) {
      const auto &dof_idx = node_idx * number_of_components + component_idx;
      (*system.rhs_vector)(dof_idx) +=
          N(0, node_idx) * load_vector[component_idx] * weight * differential;
    }
  }
}

void PhysicsProcessor::AddRadiationContributionToElementSystem(
    const Eigen::MatrixXd &N, double radiation, double weight,
    double differential, ElementSystem &system) const {
  (*system.stiffness_matrix) +=
      N.transpose() * radiation * N * weight * differential;
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

}  // namespace ffea
