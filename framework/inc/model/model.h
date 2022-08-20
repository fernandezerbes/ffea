#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <eigen3/Eigen/Dense>

#include "../assembly/assembler.h"
#include "../mesh/mesh.h"
#include "../processor/operator.h"
#include "./boundary_condition.h"
namespace ffea {

struct Model {
 public:
  Model(Mesh &mesh, const Eigen::MatrixXd &constitutive_model,
        const DifferentialOperator &differential_operator,
        const std::vector<BoundaryCondition *> &boundary_conditions,
        ConditionFunction source);

  void ProjectSolutionOnMesh(const Eigen::VectorXd &solution);

  Mesh &mesh;
  const Eigen::MatrixXd &constitutive_model;
  const DifferentialOperator &differential_operator;
  const std::vector<BoundaryCondition *> &boundary_conditions;
  ConditionFunction source;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
