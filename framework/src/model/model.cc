#include "../../inc/model/model.h"

#include <iostream>

namespace ffea {

Model::Model(Mesh& mesh) : mesh_(mesh) {}

size_t Model::number_of_dofs() const { return mesh_.number_of_dofs(); }

void Model::AddPhysicalRegion(PhysicalRegion& region) { regions_.push_back(&region); }

void Model::AddEssentialBoundaryCondition(EssentialBoundaryCondition& boundary_condition) {
  bcs_.push_back(&boundary_condition);
}

Equation Model::GetEquations(Time t) {
  auto equation = Equation(number_of_dofs());
  SetSparsity(equation);
  for (auto& region : regions_) {
    equation.Process(*region, t);
  }
  EnforceBoundaryConditions(equation, t);
  return equation;
}

void Model::SetSparsity(Equation& equation) const {
  std::cout << "Setting sparsity..." << std::endl;
  MatrixEntries<double> nonzero_entries;
  for (const auto& region : regions_) {
    for (const auto& element : region->elements()) {
      element.SetSparsity(nonzero_entries);
    }
  }
  equation.SetSparsity(nonzero_entries);
}

void Model::EnforceBoundaryConditions(Equation& equation, Time t) {
  std::cout << "Enforcing boundary conditions..." << std::endl;
  // TODO move this to the term functions
  auto& stiffness_term = equation.GetTerm<StiffnessTerm>();
  auto& rhs_term = equation.GetTerm<RhsTerm>();
  for (auto& bc : bcs_) {
    bc->Process(stiffness_term.matrix(), rhs_term.vector(), t);
  }
}

void Model::ProjectSolutionOnMesh(const Vector<double>& solution) {
  std::cout << "Projecting solution on mesh..." << std::endl;
  mesh_.SetSolutionOnDofs(solution);
}

}  // namespace ffea
