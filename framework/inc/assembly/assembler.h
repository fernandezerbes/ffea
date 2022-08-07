#ifndef FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_
#define FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_

#include <utility>

#include "../mesh/mesh.h"
#include "../model/model.h"
#include "../processor/operator.h"
#include "../model/boundary_condition.h"

namespace ffea {

class Assembler {
 public:
  static std::pair<Eigen::MatrixXd, Eigen::VectorXd> ProcessLinearSystem(
      Mesh& mesh, const Eigen::MatrixXd& constitutive_model,
      const DifferentialOperator& differential_operator,
      ConditionFunction source);
  static void EnforceBoundaryConditions(
      Eigen::MatrixXd& global_stiffness, Eigen::VectorXd& global_rhs,
      const std::vector<BoundaryCondition*>& boundary_conditions);
  static void ScatterSolution(const Eigen::VectorXd& solution);
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_
