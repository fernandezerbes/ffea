#include "../../inc/analysis/analysis.h"
#include "../../inc/model/alias.h"

#include <Eigen/IterativeLinearSolvers>
#include <iostream>

namespace ffea {

Analysis::Analysis(Model& model) : model_(model) {}

void Analysis::Solve() {
  auto number_of_dofs = model_.number_of_dofs();
  CSRMatrix<double> global_stiffness(number_of_dofs, number_of_dofs);
  Eigen::VectorXd global_rhs = Eigen::VectorXd::Zero(number_of_dofs);

  model_.SetSparsity(global_stiffness);
  model_.AddComputationalDomainContributions(global_stiffness, global_rhs);
  model_.EnforceBoundaryConditions(global_stiffness, global_rhs);

  std::cout << "Solving..." << std::endl;
  // Aparently this should work better with OpenMP enabled, but it does not
  // Eigen::ConjugateGradient<Eigen::MatrixXd, Eigen::Lower | Eigen::Upper>
  //     cg_solver;
  Eigen::ConjugateGradient<CSRMatrix<double>, Eigen::Upper> cg_solver;
  cg_solver.setTolerance(1.0e-12);
  cg_solver.compute(global_stiffness);
  Eigen::VectorXd solution;
  solution = cg_solver.solve(global_rhs);

  std::cout << "Finished solution in " << cg_solver.iterations()
            << " iterations with an estimated error of " << cg_solver.error()
            << "." << std::endl;

  model_.ProjectSolutionOnMesh(solution);
}

}  // namespace ffea
