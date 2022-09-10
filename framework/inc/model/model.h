#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <eigen3/Eigen/Dense>
#include <memory>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "../processor/operator.h"
#include "./boundary_condition.h"
#include "./computational_domain.h"
#include "./constitutive_model.h"

namespace ffea {

class Model {
 public:
  Model(Mesh &mesh);

  void AddComputationalDomain(
      const std::string &domain_name,
      const ConstitutiveModel &constitutive_model,
      const DifferentialOperator &differential_operator,
      ConditionFunction source);
  void AddNeumannBoundaryCondition(const std::string &boundary_name,
                                   ConditionFunction boundary_function);
  void AddDirichletBoundaryCondition(
      const std::string &boundary_name, ConditionFunction boundary_function,
      const std::unordered_set<size_t> &directions_to_consider,
      const EnforcementStrategy &enforcement_strategy =
          ffea::PenaltyEnforcementStrategy());
  void ProjectSolutionOnMesh(const Eigen::VectorXd &solution);
  size_t NumberOfDofs() const;

  Mesh &mesh_;
  std::vector<ComputationalDomain> computational_domains_;
  std::vector<std::unique_ptr<BoundaryCondition>> boundary_conditions_;

 private:
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
