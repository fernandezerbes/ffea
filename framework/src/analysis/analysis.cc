
#include "../../inc/analysis/analysis.h"

#include <iostream>

#include "../../inc/alias.h"
#include "../../inc/model/equation.h"

namespace ffea {

Analysis::Analysis(Model& model) : model_(model) {}

void Analysis::Solve() {
  auto number_of_dofs = model_.number_of_dofs();
  double t = 0.0;
  auto equation = model_.GetEquations(t);

  auto& stiffness_term = equation.GetTerm<StiffnessTerm>();
  auto& rhs_term = equation.GetTerm<RhsTerm>();

  std::cout << "Solving..." << std::endl;
  // Aparently this should work better with OpenMP enabled, but it does not
  // Eigen::ConjugateGradient<Matrix<double>, Eigen::Lower | Eigen::Upper>
  //     cg_solver;
  Eigen::ConjugateGradient<CSRMatrix<double>, Eigen::Upper> cg_solver;
  cg_solver.setTolerance(1.0e-12);
  cg_solver.compute(stiffness_term.matrix());
  Vector<double> solution;
  solution = cg_solver.solve(rhs_term.vector());

  std::cout << "Finished solution in " << cg_solver.iterations()
            << " iterations with an estimated error of " << cg_solver.error() << "." << std::endl;

  model_.ProjectSolutionOnMesh(solution);
}

}  // namespace ffea
