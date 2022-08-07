#include "../../inc/model/model.h"
#include <Eigen/IterativeLinearSolvers>
namespace ffea {

Model::Model(Mesh& mesh, const Eigen::MatrixXd& constitutive_model,
             const DifferentialOperator& differential_operator,
             const std::vector<BoundaryCondition*>& boundary_conditions,
             ConditionFunction source)
    : mesh_(mesh),
      constitutive_model_(constitutive_model),
      differential_operator_(differential_operator),
      boundary_conditions_(boundary_conditions),
      source_(source),
      assembler_(Assembler(mesh)) {}


Eigen::VectorXd Model::solve() const {
  auto number_of_dofs = mesh_.number_of_dofs();
  Eigen::MatrixXd global_K =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  Eigen::VectorXd global_rhs = Eigen::VectorXd::Zero(number_of_dofs);

  auto& body_elements =
      mesh_.GetElementGroup(ffea::ElementGroupType::kBodyElements, "body");

  // Compute stiffness
  for (auto& element : body_elements) {
    const auto& element_K =
        element.ComputeStiffness(constitutive_model_, differential_operator_);
    const auto& element_rhs = element.ComputeRhs(source_);
    auto nodes = element.nodes();
    const auto& dofs_map = element.GetLocalToGlobalDofIndicesMap();

    // Scatter coefficients
    for (size_t node_index = 0; node_index < nodes.size(); node_index++) {
      size_t local_i_index = 0;
      for (const auto& global_i_index : dofs_map) {
        size_t local_j_index = 0;
        for (const auto& global_j_index : dofs_map) {
          global_K(global_i_index, global_j_index) +=
              element_K(local_i_index, local_j_index);
          local_j_index++;
        }
        global_rhs(global_i_index) += element_rhs(local_i_index);
        local_i_index++;
      }
    }
  }

  // Enforce BCs
  for (auto& bc : boundary_conditions_) {
    bc->Enforce(global_K, global_rhs);
  }

  Eigen::ConjugateGradient<Eigen::MatrixXd> cg_solver;
  cg_solver.compute(global_K);
  Eigen::VectorXd solution(number_of_dofs);
  for (int i = 0; i < 100; i++) {
    solution = cg_solver.solve(global_rhs);
  }

  return solution;
}

}  // namespace ffea