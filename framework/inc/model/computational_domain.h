#ifndef FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
#define FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_

#include "../alias.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./operator.h"
#include "./physics_processor.h"

namespace ffea {

using Integrand = std::function<Matrix<double>(const Matrix<double> &, const Matrix<double> &)>;

class ComputationalDomain : public PhysicsProcessor {
 public:
  ComputationalDomain(const std::vector<Element> &elements,
                      const ConstitutiveModel &constitutive_model, Integrand integrand,
                      VectorialFunction source);

  virtual void Process(CSRMatrix<double> &system_stiffness,
                       Vector<double> &system_rhs) const override;
  virtual void Process(CSRMatrix<double> &system_stiffness, CSRMatrix<double> &system_mass,
                       Vector<double> &system_rhs) const override;
  virtual void Process(CSRMatrix<double> &system_stiffness, CSRMatrix<double> &system_mass,
                       CSRMatrix<double> &system_damping,
                       Vector<double> &system_rhs) const override;

 private:
  const ConstitutiveModel &constitutive_model_;
  Integrand integrand_;
  VectorialFunction source_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
