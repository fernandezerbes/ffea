#include "../../inc/analysis/analysis.h"

#include <Eigen/IterativeLinearSolvers>

#include "../../inc/assembly/assembler.h"

namespace ffea {

Analysis::Analysis(Model &model) : model_(model) {}

void Analysis::Solve() {
  std::cout << "Processing linear system..." << std::endl;
  auto linear_system = Assembler::ProcessLinearSystem(
      model_.mesh_, model_.constitutive_model, model_.differential_operator,
      model_.source);
  auto &global_stiffness = linear_system.first;
  auto &global_rhs = linear_system.second;
  std::cout << "Enforcing boundary conditions..." << std::endl;
  Assembler::EnforceBoundaryConditions(global_stiffness, global_rhs,
                                       model_.boundary_conditions_);

  std::cout << "Solving..." << std::endl;
  Eigen::ConjugateGradient<Eigen::MatrixXd> cg_solver;
  cg_solver.compute(global_stiffness);
  Eigen::VectorXd solution;
  solution = cg_solver.solve(global_rhs);

  model_.ProjectSolutionOnMesh(solution);
}

}  // namespace ffea
