#ifndef FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_
#define FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_

#include <utility>
#include <memory>

#include "../mesh/mesh.h"
#include "../model/boundary_condition.h"
#include "../model/model.h"
#include "../processor/operator.h"

namespace ffea {

class Assembler {
 public:
  static std::pair<Eigen::MatrixXd, Eigen::VectorXd> ProcessLinearSystem(
      Mesh& mesh, const ConstitutiveModel& constitutive_model,
      const DifferentialOperator& differential_operator,
      ConditionFunction source);
  static void EnforceBoundaryConditions(
      Eigen::MatrixXd& global_stiffness, Eigen::VectorXd& global_rhs,
      const std::vector<std::unique_ptr<BoundaryCondition>>&
          boundary_conditions);
  static void ScatterSolution(const Eigen::VectorXd& solution);
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_
