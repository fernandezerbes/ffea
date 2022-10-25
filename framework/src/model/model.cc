#include "../../inc/model/model.h"

#include <iostream>

namespace ffea {

Model::Model(Mesh& mesh) : mesh_(mesh), domains_(), bcs_() {}

size_t Model::number_of_dofs() const { return mesh_.number_of_dofs(); }

void Model::AddComputationalDomain(const std::string& domain_name,
                                   const ConstitutiveModel& constitutive_model,
                                   Integrand integrand,
                                   VectorialFunction source) {
  const auto& elements = mesh_.element_group(domain_name);
  domains_.emplace_back(elements, constitutive_model, integrand, source);
}

void Model::AddNaturalBoundaryCondition(const std::string& boundary_name,
                                        VectorialFunction load,
                                        VectorialFunction radiation) {
  const auto& elements = mesh_.element_group(boundary_name);
  bcs_.push_back(std::make_unique<ffea::NaturalBoundaryCondition>(
      elements, load, radiation));
}

void Model::AddEssentialBoundaryCondition(
    const std::string& boundary_name, VectorialFunction condition,
    const std::unordered_set<size_t>& components_to_consider,
    const EnforcementStrategy& strategy) {
  const auto& elements = mesh_.element_group(boundary_name);
  bcs_.push_back(std::make_unique<ffea::EssentialBoundaryCondition>(
      elements, condition, components_to_consider, strategy));
}

void Model::SetSparsity(CSRMatrix<double>& global_stiffness) const {
  std::cout << "Setting sparsity..." << std::endl;
  MatrixEntries<double> nonzero_entries;
  for (const auto& domain : domains_) {
    domain.SetSparsity(nonzero_entries);
  }
  global_stiffness.setFromTriplets(nonzero_entries.begin(),
                                   nonzero_entries.end());
}

void Model::AddComputationalDomainContributions(
    CSRMatrix<double>& global_stiffness, Vector<double>& global_rhs) const {
  std::cout << "Processing linear system..." << std::endl;
  for (const auto& domain : domains_) {
    domain.AddContribution(global_stiffness, global_rhs);
  }
}

void Model::EnforceBoundaryConditions(CSRMatrix<double>& global_stiffness,
                                      Vector<double>& global_rhs) const {
  std::cout << "Enforcing boundary conditions..." << std::endl;
  for (auto& bc : bcs_) {
    bc->Enforce(global_stiffness, global_rhs);
  }
}

void Model::ProjectSolutionOnMesh(const Vector<double>& solution) {
  std::cout << "Projecting solution on mesh..." << std::endl;
  mesh_.SetSolutionOnDofs(solution);
}

}  // namespace ffea
