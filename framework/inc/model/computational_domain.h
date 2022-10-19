#ifndef FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
#define FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <string>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./types.h"
#include "./operator.h"

namespace ffea {

class ComputationalDomain {
 public:
  ComputationalDomain(const std::vector<Element> &elements,
                      const ConstitutiveModel &constitutive_model,
                      Integrand integrand, ConditionFunction source);

  void AddContribution(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const;

 private:
  const std::vector<Element> &elements_;
  const ConstitutiveModel &constitutive_model_;
  Integrand integrand_;
  ConditionFunction source_;
};

const Integrand elasticity_integrand_2D =
    [](const Eigen::MatrixXd &dN_global,
       const Eigen::MatrixXd &C) -> Eigen::MatrixXd {
  const auto &B = linear_B_operator_2D(dN_global);
  return B.transpose() * C * B;
};

const Integrand elasticity_integrand_3D =
    [](const Eigen::MatrixXd &dN_global,
       const Eigen::MatrixXd &C) -> Eigen::MatrixXd {
  const auto &B = linear_B_operator_3D(dN_global);
  return B.transpose() * C * B;
};

const Integrand quasi_harmonic_integrand =
    [](const Eigen::MatrixXd &dN_global,
       const Eigen::MatrixXd &C) -> Eigen::MatrixXd {
  const auto &B = gradient_operator(dN_global);
  return B.transpose() * C * B;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
