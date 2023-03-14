#ifndef FFEA_FRAMEWORK_INC_MODEL_PHYSICSPROCESSOR_H_
#define FFEA_FRAMEWORK_INC_MODEL_PHYSICSPROCESSOR_H_

#include "../alias.h"
#include "../mesh/element.h"

namespace ffea {

class PhysicsProcessor {
 public:
  PhysicsProcessor(const std::vector<Element> &elements);

  virtual void Process(CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs) const = 0;
  virtual void Process(CSRMatrix<double> &system_stiffness, CSRMatrix<double> &system_mass,
                       Vector<double> &system_rhs) const = 0;
  virtual void Process(CSRMatrix<double> &system_stiffness, CSRMatrix<double> &system_mass,
                       CSRMatrix<double> &system_damping, Vector<double> &system_rhs) const = 0;
  void SetSparsity(MatrixEntries<double> &nonzero_entries) const;

 protected:
  void AddLoadContribution(const std::vector<double> &load_vector, const Matrix<double> &N,
                           double weight, double differential, Vector<double> &element_rhs) const;
  void Scatter(const std::vector<size_t> &dofs_tags, const Matrix<double> &element_stiffness,
               const Vector<double> &element_rhs, CSRMatrix<double> &system_stiffness,
               Vector<double> &system_rhs) const;
  void Scatter(const std::vector<size_t> &dofs_tags, const Matrix<double> &element_stiffness,
               CSRMatrix<double> &system_stiffness) const;
  void Scatter(const std::vector<size_t> &dofs_tags, const Vector<double> &element_rhs,
               Vector<double> &system_rhs) const;
  const std::vector<Element> &elements_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_PHYSICSPROCESSOR_H_
