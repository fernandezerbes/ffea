#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <memory>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "../alias.h"
#include "./boundary_condition.h"
#include "./computational_domain.h"
#include "./constitutive_model.h"

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
      const EnforcementStrategy &strategy = ffea::PenaltyEnforcementStrategy());
  void SetSparsity(CSRMatrix<double> &global_stiffness) const;
  void AddComputationalDomainContributions(CSRMatrix<double> &global_stiffness,
                                           Vector<double> &global_rhs) const;
  void EnforceBoundaryConditions(CSRMatrix<double> &global_stiffness,
                                 Vector<double> &global_rhs) const;
  void ProjectSolutionOnMesh(const Vector<double> &solution);

 private:
  Mesh &mesh_;
  std::vector<ComputationalDomain> domains_;
  std::vector<std::unique_ptr<BoundaryCondition>> bcs_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
