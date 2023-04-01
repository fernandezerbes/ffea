#include "../../inc/model/equation.h"

// #include <stdexcept>

namespace ffea {

Equation::Equation(size_t number_of_dofs) : number_of_dofs_(number_of_dofs), terms_() {
  terms_.push_back(std::make_unique<StiffnessTerm>(number_of_dofs));
  terms_.push_back(std::make_unique<RhsTerm>(number_of_dofs));
}

void Equation::SetSparsity(const MatrixEntries<double>& nonzero_entries) const {
  for (auto& term : terms_) {
    term->SetSparsity(nonzero_entries);
  }
}

void Equation::Process(PhysicalRegion& region, Time t) {
  for (auto& element : region.elements()) {
    PrepareElementMatrices(element);
    Integrate(region, element, t);
    Scatter(element);
  }
}

void Equation::AddDampingTerm() {
  if (HasTerm<DampingTerm>()) {
    throw std::runtime_error("The equation already has a damping term.");
  }

  terms_.push_back(std::make_unique<DampingTerm>(number_of_dofs_));
}

void Equation::AddMassTerm() {
  if (HasTerm<MassTerm>()) {
    throw std::runtime_error("The equation already has a damping term.");
  }

  terms_.push_back(std::make_unique<MassTerm>(number_of_dofs_));
}

void Equation::Integrate(PhysicalRegion& region, Element& element, Time t) {
  for (size_t ip_idx = 0; ip_idx < element.number_of_integration_points(); ip_idx++) {
    for (auto& term : terms_) {
      term->Process(region, element, ip_idx, t);
    }
  }
}

void Equation::Scatter(Element& element) {
  const auto& dofs_tags = element.dof_tags();
  for (size_t i_dof_idx = 0; i_dof_idx < dofs_tags.size(); i_dof_idx++) {
    for (auto& term : terms_) {
      term->Scatter(i_dof_idx, dofs_tags);
    }
  }
}

void Equation::PrepareElementMatrices(Element& element) {
  element.ResetCache();
  for (auto& term : terms_) {
    term->PrepareElementMatrices(element.number_of_dofs());
  }
}

void Equation::ResetGlobalData() {
  for (auto& term : terms_) {
    term->ResetGlobalData();
  }
}

MatricialEquationTerm::MatricialEquationTerm(size_t number_of_dofs) {
  matrix_ = CSRMatrix<double>(number_of_dofs, number_of_dofs);
}

void EquationTerm::SetSparsity(MatrixEntries<double> nonzero_entries) {}

void MatricialEquationTerm::SetSparsity(MatrixEntries<double> nonzero_entries) {
  matrix_.setFromTriplets(nonzero_entries.begin(), nonzero_entries.end());
}

void MatricialEquationTerm::Scatter(size_t i_dof_idx, const std::vector<size_t>& dofs_tags) {
  const auto& i_dof_tag = dofs_tags[i_dof_idx];
  for (size_t j_dof_idx = i_dof_idx; j_dof_idx < dofs_tags.size(); j_dof_idx++) {
    const auto& value = element_matrix_.selfadjointView<Eigen::Upper>()(i_dof_idx, j_dof_idx);
    const auto& j_dof_tag = dofs_tags[j_dof_idx];
    if (i_dof_tag <= j_dof_tag) {
      matrix_.coeffRef(i_dof_tag, j_dof_tag) += value;
    } else {
      matrix_.coeffRef(j_dof_tag, i_dof_tag) += value;
    }
  }
}

void MatricialEquationTerm::PrepareElementMatrices(size_t number_of_dofs) {
  element_matrix_ = Matrix<double>::Zero(number_of_dofs, number_of_dofs);
}

void MatricialEquationTerm::ResetGlobalData() { matrix_.setZero(); }

Matrix<double>& MatricialEquationTerm::element_matrix() { return element_matrix_; }

CSRMatrix<double>& MatricialEquationTerm::matrix() { return matrix_; }

VectorialEquationTerm::VectorialEquationTerm(size_t number_of_dofs) {
  vector_ = Vector<double>::Zero(number_of_dofs);
}

void VectorialEquationTerm::Scatter(size_t i_dof_idx, const std::vector<size_t>& dofs_tags) {
  const auto& i_dof_tag = dofs_tags[i_dof_idx];
  vector_(i_dof_tag) += element_vector_(i_dof_idx);
}

void VectorialEquationTerm::PrepareElementMatrices(size_t number_of_dofs) {
  element_vector_ = Vector<double>::Zero(number_of_dofs);
}

void VectorialEquationTerm::ResetGlobalData() { vector_.setZero(); }

Vector<double>& VectorialEquationTerm::element_vector() { return element_vector_; }

Vector<double>& VectorialEquationTerm::vector() { return vector_; }

StiffnessTerm::StiffnessTerm(size_t number_of_dofs) : MatricialEquationTerm(number_of_dofs) {}

void StiffnessTerm::Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                            Time t) {
  region.Contribute(*this, element, integration_point_idx, t);
}

DampingTerm::DampingTerm(size_t number_of_dofs) : MatricialEquationTerm(number_of_dofs) {}

void DampingTerm::Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                          Time t) {
  region.Contribute(*this, element, integration_point_idx, t);
}

MassTerm::MassTerm(size_t number_of_dofs) : MatricialEquationTerm(number_of_dofs) {}

void MassTerm::Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                       Time t) {
  region.Contribute(*this, element, integration_point_idx, t);
}

RhsTerm::RhsTerm(size_t number_of_dofs) : VectorialEquationTerm(number_of_dofs) {}

void RhsTerm::Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                      Time t) {
  region.Contribute(*this, element, integration_point_idx, t);
}

}  // namespace ffea
