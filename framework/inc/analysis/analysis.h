#ifndef FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_
#define FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_

#include <eigen3/Eigen/Dense>

#include "../model/model.h"

namespace ffea {

class Analysis {
 public:
  Analysis(Model &model);

  Eigen::VectorXd solve();

 private:
  Model &model_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_

// Eigen::VectorXd Model::solve() const {
//   auto &linear_system = assembler_.ProcessLinearSystem();
//   auto &global_stiffness = linear_system.first;
//   auto &global_rhs = linear_system.second;

//   // Enforce BCs
//   for (auto& bc : boundary_conditions_) {
//     bc->Enforce(global_stiffness, global_rhs);
//   }

//   Eigen::ConjugateGradient<Eigen::MatrixXd> cg_solver;
//   cg_solver.compute(global_stiffness);
//   Eigen::VectorXd solution(mesh_.number_of_dofs());
//   for (int i = 0; i < 100; i++) {
//     solution = cg_solver.solve(global_rhs);
//   }

//   return solution;
// }