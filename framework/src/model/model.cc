#include "../../inc/model/model.h"

#include <iostream>

namespace ffea {

Model::Model(Mesh& mesh) : mesh_(mesh), domains_(), bcs_() {}

size_t Model::number_of_dofs() const { return mesh_.number_of_dofs(); }

void Model::AddComputationalDomain(const std::string& domain_name,
                                   const ConstitutiveModel& constitutive_model,
                                   Integrand integrand) {
  AddComputationalDomain(domain_name, constitutive_model, integrand, nullptr);
}

void Model::AddComputationalDomain(const std::string& domain_name,
                                   const ConstitutiveModel& constitutive_model, Integrand integrand,
                                   VectorialFunction source) {
  const auto& elements = mesh_.element_group(domain_name);
  domains_.push_back(
      std::make_unique<ffea::ComputationalDomain>(elements, constitutive_model, integrand, source));
}

void Model::AddNaturalBoundaryCondition(const std::string& boundary_name, VectorialFunction load) {
  AddNaturalBoundaryCondition(boundary_name, load, nullptr);
}

void Model::AddNaturalBoundaryCondition(const std::string& boundary_name, VectorialFunction load,
                                        VectorialFunction radiation) {
  const auto& elements = mesh_.element_group(boundary_name);
  // Natural boundary conditions must be processed first, add to front
  bcs_.push_front(std::make_unique<ffea::NaturalBoundaryCondition>(elements, load, radiation));
}

void Model::AddEssentialBoundaryCondition(const std::string& boundary_name,
                                          VectorialFunction condition,
                                          const std::unordered_set<size_t>& components_to_consider,
                                          const EnforcementStrategy& strategy) {
  const auto& elements = mesh_.element_group(boundary_name);
  // Essential boundary conditions must be processed last, add to back
  bcs_.push_back(std::make_unique<ffea::EssentialBoundaryCondition>(
      elements, condition, components_to_consider, strategy));
}

void Model::SetSparsity(CSRMatrix<double>& system_stiffness) const {
  std::cout << "Setting sparsity..." << std::endl;
  MatrixEntries<double> nonzero_entries;
  for (const auto& domain : domains_) {
    domain->SetSparsity(nonzero_entries);
  }
  system_stiffness.setFromTriplets(nonzero_entries.begin(), nonzero_entries.end());
}

void Model::AddComputationalDomainContributions(CSRMatrix<double>& system_stiffness,
                                                Vector<double>& system_rhs) const {
  std::cout << "Processing linear system..." << std::endl;
  for (const auto& domain : domains_) {
    domain->Process(system_stiffness, system_rhs);
  }
}

void Model::EnforceBoundaryConditions(CSRMatrix<double>& system_stiffness,
                                      Vector<double>& system_rhs) const {
  std::cout << "Enforcing boundary conditions..." << std::endl;
  for (auto& bc : bcs_) {
    bc->Process(system_stiffness, system_rhs);
  }
}

void Model::ProjectSolutionOnMesh(const Vector<double>& solution) {
  std::cout << "Projecting solution on mesh..." << std::endl;
  mesh_.SetSolutionOnDofs(solution);
}

}  // namespace ffea
