
#include "../../inc/analysis/analysis.h"

#include <iostream>

#include "../../inc/alias.h"

namespace ffea {

Analysis::Analysis(Model& model) : model_(model) {}

void Analysis::Solve() {
  auto number_of_dofs = model_.number_of_dofs();
  CSRMatrix<double> system_stiffness(number_of_dofs, number_of_dofs);
  Vector<double> system_rhs = Vector<double>::Zero(number_of_dofs);

  model_.SetSparsity(system_stiffness);
  model_.AddComputationalDomainContributions(system_stiffness, system_rhs);
  model_.EnforceBoundaryConditions(system_stiffness, system_rhs);

  std::cout << "Solving..." << std::endl;
  // Aparently this should work better with OpenMP enabled, but it does not
  // Eigen::ConjugateGradient<Matrix<double>, Eigen::Lower | Eigen::Upper>
  //     cg_solver;
  Eigen::ConjugateGradient<CSRMatrix<double>, Eigen::Upper> cg_solver;
  cg_solver.setTolerance(1.0e-12);
  cg_solver.compute(system_stiffness);
  Vector<double> solution;
  solution = cg_solver.solve(system_rhs);

  std::cout << "Finished solution in " << cg_solver.iterations()
            << " iterations with an estimated error of " << cg_solver.error()
            << "." << std::endl;

  model_.ProjectSolutionOnMesh(solution);
}

}  // namespace ffea
