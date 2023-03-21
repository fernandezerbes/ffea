#ifndef FFEA_FRAMEWORK_MODEL_EQUATION_H_
#define FFEA_FRAMEWORK_MODEL_EQUATION_H_

#include <memory>
#include <stdexcept>
#include <vector>

#include "../alias.h"
#include "../mesh/element.h"
#include "./physical_region.h"

namespace ffea {

class PhysicalRegion;
class EquationTerm;

class Equation {
 public:
  explicit Equation(size_t number_of_dofs);
  void SetSparsity(MatrixEntries<double> nonzero_entries) const;
  void Process(PhysicalRegion& regions, double t);
  void AddDampingTerm();
  void AddMassTerm();

  template <typename T>
  T& GetTerm() const;

 private:
  template <typename T>
  bool HasTerm() const;

  void Integrate(PhysicalRegion& region, Element& element, double t);
  void Scatter(Element& element);
  void PrepareElementMatrices(Element& element);
  void ResetGlobalData();

  size_t number_of_dofs_;
  std::vector<std::unique_ptr<EquationTerm>> terms_;
};

template <typename T>
T& Equation::GetTerm() const {
  for (const auto& term : terms_) {
    if (typeid(*term) == typeid(T)) {
      return static_cast<T&>(*term);
    }
  }

  std::string type_name = typeid(T).name();
  throw std::runtime_error("The equation doesnt have a term of type " + type_name);
}

template <typename T>
bool Equation::HasTerm() const {
  for (const auto& term : terms_) {
    if (typeid(*term) == typeid(T)) {
      return true;
    }
  }
  return false;
}

class EquationTerm {
 public:
  virtual void SetSparsity(MatrixEntries<double> nonzero_entries);
  virtual void Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                       double t) = 0;
  virtual void Scatter(size_t i_dof_idx, const std::vector<size_t>& dof_tags) = 0;
  virtual void PrepareElementMatrices(size_t number_of_dofs) = 0;
  virtual void ResetGlobalData() = 0;
};

class MatricialEquationTerm : public EquationTerm {
 public:
  explicit MatricialEquationTerm(size_t number_of_dofs);
  virtual void SetSparsity(MatrixEntries<double> nonzero_entries) override;
  virtual void Scatter(size_t i_dof_idx, const std::vector<size_t>& dofs_tags) override;
  virtual void PrepareElementMatrices(size_t number_of_dofs) override;
  virtual void ResetGlobalData() override;
  Matrix<double>& element_matrix();
  CSRMatrix<double>& matrix();

 private:
  Matrix<double> element_matrix_;
  CSRMatrix<double> matrix_;
};

class VectorialEquationTerm : public EquationTerm {
 public:
  explicit VectorialEquationTerm(size_t number_of_dofs);
  virtual void Scatter(size_t i_dof_idx, const std::vector<size_t>& dof_tags) override;
  virtual void PrepareElementMatrices(size_t number_of_dofs) override;
  virtual void ResetGlobalData() override;
  Vector<double>& element_vector();
  Vector<double>& vector();

 private:
  Vector<double> element_vector_;
  Vector<double> vector_;
};

class StiffnessTerm : public MatricialEquationTerm {
 public:
  explicit StiffnessTerm(size_t number_of_dofs);
  virtual void Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                       double t) override;
};

class DampingTerm : public MatricialEquationTerm {
 public:
  explicit DampingTerm(size_t number_of_dofs);
  virtual void Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                       double t) override;
};

class MassTerm : public MatricialEquationTerm {
 public:
  explicit MassTerm(size_t number_of_dofs);
  virtual void Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                       double t) override;
};

class RhsTerm : public VectorialEquationTerm {
 public:
  explicit RhsTerm(size_t number_of_dofs);
  virtual void Process(PhysicalRegion& region, Element& element, size_t integration_point_idx,
                       double t) override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_EQUATION_H_
