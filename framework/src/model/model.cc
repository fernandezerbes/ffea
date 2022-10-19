#include "../../inc/model/model.h"

#include <iostream>

namespace ffea {

Model::Model(Mesh& mesh)
    : mesh_(mesh), computational_domains_(), boundary_conditions_() {}

void Model::AddComputationalDomain(const std::string& domain_name,
                                   const ConstitutiveModel& constitutive_model,
                                   Integrand integrand,
                                   ConditionFunction source) {
  const auto& domain_elements = mesh_.element_group(domain_name);
  computational_domains_.emplace_back(domain_elements, constitutive_model,
                                      integrand, source);
}

void Model::AddNaturalBoundaryCondition(const std::string& boundary_name,
                                        ConditionFunction load,
                                        ConditionFunction radiation) {
  const auto& boundary_elements = mesh_.element_group(boundary_name);
  boundary_conditions_.push_back(
      std::make_unique<ffea::NaturalBoundaryCondition>(boundary_elements, load,
                                                       radiation));
}

void Model::AddEssentialBoundaryCondition(
    const std::string& boundary_name, ConditionFunction condition,
    const std::unordered_set<size_t>& components_to_consider,
    const EnforcementStrategy& enforcement_strategy) {
  const auto& boundary_elements = mesh_.element_group(boundary_name);
  boundary_conditions_.push_back(
      std::make_unique<ffea::EssentialBoundaryCondition>(
          boundary_elements, condition, components_to_consider,
          enforcement_strategy));
}

size_t Model::number_of_dofs() const { return mesh_.number_of_dofs(); }

void Model::ProjectSolutionOnMesh(const Eigen::VectorXd& solution) {
  std::cout << "Projecting solution on mesh..." << std::endl;
  mesh_.SetSolutionOnDofs(solution);
}

void Model::AddComputationalDomainsContributions(
    Eigen::MatrixXd& global_stiffness, Eigen::VectorXd& global_rhs) const {
  std::cout << "Processing linear system..." << std::endl;
  for (const auto& domain : computational_domains_) {
    domain.AddContribution(global_stiffness, global_rhs);
  }
}

void Model::EnforceBoundaryConditions(Eigen::MatrixXd& global_stiffness,
                                      Eigen::VectorXd& global_rhs) const {
  std::cout << "Enforcing boundary conditions..." << std::endl;
  for (auto& bc : boundary_conditions_) {
    bc->Enforce(global_stiffness, global_rhs);
  }
}

}  // namespace ffea
