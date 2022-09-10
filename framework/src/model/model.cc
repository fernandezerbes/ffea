#include "../../inc/model/model.h"

namespace ffea {

Model::Model(Mesh& mesh)
    : mesh_(mesh),
      computational_domains_(),
      boundary_conditions_() {}

void Model::AddComputationalDomain(
    const std::string& domain_name, const ConstitutiveModel& constitutive_model,
    const DifferentialOperator& differential_operator,
    ConditionFunction source) {
  computational_domains_.emplace_back(mesh_, domain_name, constitutive_model,
                                      differential_operator, source);
}

void Model::AddNeumannBoundaryCondition(const std::string& boundary_name,
                                        ConditionFunction boundary_function) {
  boundary_conditions_.push_back(
      std::make_unique<ffea::NeumannBoundaryCondition>(mesh_, boundary_name,
                                                       boundary_function));
}

void Model::AddDirichletBoundaryCondition(
    const std::string& boundary_name, ConditionFunction boundary_function,
    const std::unordered_set<size_t>& directions_to_consider,
    const EnforcementStrategy& enforcement_strategy) {
  boundary_conditions_.push_back(
      std::make_unique<ffea::DirichletBoundaryCondition>(
          mesh_, boundary_name, boundary_function, directions_to_consider,
          enforcement_strategy));
}

size_t Model::NumberOfDofs() const {
  return mesh_.number_of_dofs();
}

void Model::ProjectSolutionOnMesh(const Eigen::VectorXd& solution) {
  mesh_.SetSolutionOnDofs(solution);
}

}  // namespace ffea
