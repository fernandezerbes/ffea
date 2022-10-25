#ifndef FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
#define FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "../alias.h"
#include "./operator.h"

namespace ffea {

class ComputationalDomain {
 public:
  ComputationalDomain(const std::vector<Element> &elements,
                      const ConstitutiveModel &constitutive_model,
                      Integrand integrand, VectorialFunction source);

  void SetSparsity(MatrixEntries<double> &nonzero_entries) const;
  void AddContribution(CSRMatrix<double> &global_stiffness,
                       Vector<double> &global_rhs) const;

 private:
  const std::vector<Element> &elements_;
  const ConstitutiveModel &constitutive_model_;
  Integrand integrand_;
  VectorialFunction source_;
};

const Integrand elasticity_integrand_2D =
    [](const Matrix<double> &dN_global,
       const Matrix<double> &C) -> Matrix<double> {
  const auto &B = linear_B_operator_2D(dN_global);
  return B.transpose() * C * B;
};

const Integrand elasticity_integrand_3D =
    [](const Matrix<double> &dN_global,
       const Matrix<double> &C) -> Matrix<double> {
  const auto &B = linear_B_operator_3D(dN_global);
  return B.transpose() * C * B;
};

const Integrand quasi_harmonic_integrand =
    [](const Matrix<double> &dN_global,
       const Matrix<double> &C) -> Matrix<double> {
  const auto &B = gradient_operator(dN_global);
  return B.transpose() * C * B;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
