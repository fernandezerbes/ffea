#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <list>
#include <memory>

#include "../alias.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./boundary_condition.h"
#include "./computational_domain.h"
#include "./constitutive_model.h"
#include "./physics_processor.h"

namespace ffea {

class Model {
 public:
  explicit Model(Mesh &mesh);

  size_t number_of_dofs() const;

  void AddComputationalDomain(const std::string &domain_name,
                              const ConstitutiveModel &constitutive_model,
                              Integrand integrand);
  void AddComputationalDomain(const std::string &domain_name,
                              const ConstitutiveModel &constitutive_model,
                              Integrand integrand, VectorialFunction source);
  void AddNaturalBoundaryCondition(const std::string &boundary_name,
                                   VectorialFunction load);
  void AddNaturalBoundaryCondition(const std::string &boundary_name,
                                   VectorialFunction load,
                                   VectorialFunction radiation);
  void AddEssentialBoundaryCondition(
      const std::string &boundary_name, VectorialFunction condition,
      const std::unordered_set<size_t> &components_to_consider,
      const EnforcementStrategy &strategy = ffea::PenaltyEnforcementStrategy());
  void SetSparsity(CSRMatrix<double> &system_stiffness) const;
  void AddComputationalDomainContributions(CSRMatrix<double> &system_stiffness,
                                           Vector<double> &system_rhs) const;
  void EnforceBoundaryConditions(CSRMatrix<double> &system_stiffness,
                                 Vector<double> &system_rhs) const;
  void ProjectSolutionOnMesh(const Vector<double> &solution);

 private:
  Mesh &mesh_;
  std::vector<std::unique_ptr<PhysicsProcessor>> domains_;
  std::list<std::unique_ptr<PhysicsProcessor>> bcs_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
