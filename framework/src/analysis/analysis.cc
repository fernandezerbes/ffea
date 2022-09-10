#include "../../inc/analysis/analysis.h"

#include <Eigen/IterativeLinearSolvers>

namespace ffea {

Analysis::Analysis(Model &model) : model_(model) {}

void Analysis::Solve() {
  std::cout << "Processing linear system..." << std::endl;

  auto number_of_dofs = model_.NumberOfDofs();
  Eigen::MatrixXd global_stiffness =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  Eigen::VectorXd global_rhs = Eigen::VectorXd::Zero(number_of_dofs);

  for (const auto& domain : model_.computational_domains_) {
    domain.AddContribution(global_stiffness, global_rhs);
  }

  std::cout << "Enforcing boundary conditions..." << std::endl;
  for (auto& bc : model_.boundary_conditions_) {
    bc->Enforce(global_stiffness, global_rhs);
  }

  std::cout << "Solving..." << std::endl;
  Eigen::ConjugateGradient<Eigen::MatrixXd> cg_solver;
  cg_solver.compute(global_stiffness);
  Eigen::VectorXd solution;
  solution = cg_solver.solve(global_rhs);

  model_.ProjectSolutionOnMesh(solution);
}

}  // namespace ffea
