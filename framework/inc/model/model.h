#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <eigen3/Eigen/Dense>

#include "../assembly/assembler.h"
#include "../mesh/mesh.h"
#include "../processor/operator.h"
#include "./boundary_condition.h"
namespace ffea {

class Model {
 public:
  Model(Mesh &mesh, const Eigen::MatrixXd &constitutive_model,
        const DifferentialOperator &differential_operator,
        const std::vector<BoundaryCondition *> &boundary_conditions,
        ConditionFunction source);

  Eigen::VectorXd solve() const;

 private:
  Mesh &mesh_;
  const Eigen::MatrixXd &constitutive_model_;
  const DifferentialOperator &differential_operator_;
  const std::vector<BoundaryCondition *> &boundary_conditions_;
  ConditionFunction source_;
  const Assembler &assembler_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
