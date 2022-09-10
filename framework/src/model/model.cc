#include "../../inc/model/model.h"

namespace ffea {

Model::Model(Mesh& mesh)
    : mesh_(mesh), computational_domains_(), boundary_conditions_() {}

void Model::AddComputationalDomain(
    const std::string& domain_name, const ConstitutiveModel& constitutive_model,
    const DifferentialOperator& differential_operator,
    ConditionFunction source) {
  const auto& domain_elements = mesh_.GetElementGroup(domain_name);
  computational_domains_.emplace_back(domain_elements, constitutive_model,
                                      differential_operator, source);
}

void Model::AddNeumannBoundaryCondition(const std::string& boundary_name,
                                        ConditionFunction boundary_function) {
  const auto& boundary_elements = mesh_.GetElementGroup(boundary_name);
  boundary_conditions_.push_back(
      std::make_unique<ffea::NeumannBoundaryCondition>(boundary_elements,
                                                       boundary_function));
}

void Model::AddDirichletBoundaryCondition(
    const std::string& boundary_name, ConditionFunction boundary_function,
    const std::unordered_set<size_t>& directions_to_consider,
    const EnforcementStrategy& enforcement_strategy) {
  const auto& boundary_elements = mesh_.GetElementGroup(boundary_name);
  boundary_conditions_.push_back(
      std::make_unique<ffea::DirichletBoundaryCondition>(
          boundary_elements, boundary_function, directions_to_consider,
          enforcement_strategy));
}

size_t Model::NumberOfDofs() const { return mesh_.number_of_dofs(); }

void Model::ProjectSolutionOnMesh(const Eigen::VectorXd& solution) {
  mesh_.SetSolutionOnDofs(solution);
}

}  // namespace ffea
