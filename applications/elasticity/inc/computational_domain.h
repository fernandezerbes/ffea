#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_COMPUTATIONALDOMAIN_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_COMPUTATIONALDOMAIN_H_

#include "../../../framework/inc/alias.h"
#include "../../../framework/inc/model/computational_domain.h"
#include "./operator.h"

namespace ffea::app {

const Integrand elasticity_integrand_2D = [](const Matrix<double> &dN_global,
                                             const Matrix<double> &C) -> Matrix<double> {
  const auto &B = linear_B_operator_2D(dN_global);
  return B.transpose() * C * B;
};

const Integrand elasticity_integrand_3D = [](const Matrix<double> &dN_global,
                                             const Matrix<double> &C) -> Matrix<double> {
  const auto &B = linear_B_operator_3D(dN_global);
  return B.transpose() * C * B;
};

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_COMPUTATIONALDOMAIN_H_
