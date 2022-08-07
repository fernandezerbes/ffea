#include "../../inc/analysis/analysis.h"

#include <Eigen/IterativeLinearSolvers>

#include "../../inc/assembly/assembler.h"

namespace ffea {

Analysis::Analysis(Model &model) : model_(model) {}

Eigen::VectorXd Analysis::Solve() {
  auto linear_system = Assembler::ProcessLinearSystem(
      model_.mesh, model_.constitutive_model, model_.differential_operator,
      model_.source);
  auto &global_stiffness = linear_system.first;
  auto &global_rhs = linear_system.second;
  Assembler::EnforceBoundaryConditions(global_stiffness, global_rhs,
                                       model_.boundary_conditions);

  Eigen::ConjugateGradient<Eigen::MatrixXd> cg_solver;
  cg_solver.compute(global_stiffness);
  Eigen::VectorXd solution;
  for (int i = 0; i < 100; i++) {
    solution = cg_solver.solve(global_rhs);
  }

  return solution;
}

}  // namespace ffea
