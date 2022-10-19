#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <eigen3/Eigen/Dense>
#include <memory>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./boundary_condition.h"
#include "./computational_domain.h"
#include "./constitutive_model.h"
#include "./types.h"

namespace ffea {

class Model {
 public:
  explicit Model(Mesh &mesh);
  
  size_t number_of_dofs() const;

  void AddComputationalDomain(const std::string &domain_name,
                              const ConstitutiveModel &constitutive_model,
                              Integrand integrand,
                              ConditionFunction source = nullptr);
  void AddNaturalBoundaryCondition(const std::string &boundary_name,
                                   ConditionFunction load,
                                   ConditionFunction radiation);
  void AddEssentialBoundaryCondition(
      const std::string &boundary_name, ConditionFunction condition,
      const std::unordered_set<size_t> &components_to_consider,
      const EnforcementStrategy &strategy =
          ffea::PenaltyEnforcementStrategy());
  void EnforceBoundaryConditions(Eigen::MatrixXd &global_stiffness,
                                 Eigen::VectorXd &global_rhs) const;
  void AddComputationalDomainsContributions(Eigen::MatrixXd &global_stiffness,
                                            Eigen::VectorXd &global_rhs) const;
  void ProjectSolutionOnMesh(const Eigen::VectorXd &solution);

 private:
  Mesh &mesh_;
  std::vector<ComputationalDomain> computational_domains_;
  std::vector<std::unique_ptr<BoundaryCondition>> boundary_conditions_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
