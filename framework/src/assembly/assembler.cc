#include "../../inc/assembly/assembler.h"

namespace ffea {

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Assembler::ProcessLinearSystem(
    Mesh& mesh, const Eigen::MatrixXd& constitutive_model,
    const DifferentialOperator& differential_operator,
    ConditionFunction source) {
  auto number_of_dofs = mesh.number_of_dofs();
  Eigen::MatrixXd global_stiffness =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  Eigen::VectorXd global_rhs = Eigen::VectorXd::Zero(number_of_dofs);

  auto& body_elements = mesh.GetElementGroup("surface");

  // Compute stiffness
  for (auto& element : body_elements) {
    const auto& element_K =
        element.ComputeStiffness(constitutive_model, differential_operator);
    const auto& element_rhs = element.ComputeRhs(source);
    auto nodes = element.nodes();
    const auto& dofs_map = element.GetLocalToGlobalDofIndicesMap();

    // Scatter coefficients
    for (size_t node_index = 0; node_index < nodes.size(); node_index++) {
      size_t local_i_index = 0;
      for (const auto& global_i_index : dofs_map) {
        size_t local_j_index = 0;
        for (const auto& global_j_index : dofs_map) {
          global_stiffness(global_i_index, global_j_index) +=
              element_K(local_i_index, local_j_index);
          local_j_index++;
        }
        global_rhs(global_i_index) += element_rhs(local_i_index);
        local_i_index++;
      }
    }
  }

  return {global_stiffness, global_rhs};
}

void Assembler::EnforceBoundaryConditions(
    Eigen::MatrixXd& global_stiffness, Eigen::VectorXd& global_rhs,
    const std::vector<std::unique_ptr<BoundaryCondition>>&
        boundary_conditions) {
  for (auto& bc : boundary_conditions) {
    bc->Enforce(global_stiffness, global_rhs);
  }
}

void Assembler::ScatterSolution(const Eigen::VectorXd& solution) {}

}  // namespace ffea
