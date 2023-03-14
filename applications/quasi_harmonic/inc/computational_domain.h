#ifndef FFEA_APPLICATIONS_QUASIHARMONIC_INC_COMPUTATIONALDOMAIN_H_
#define FFEA_APPLICATIONS_QUASIHARMONIC_INC_COMPUTATIONALDOMAIN_H_

#include "../../../framework/inc/alias.h"
#include "../../../framework/inc/model/computational_domain.h"
#include "../../../framework/inc/model/operator.h"

namespace ffea::app {

const Integrand quasi_harmonic_integrand = [](const Matrix<double> &dN_global,
                                              const Matrix<double> &C) -> Matrix<double> {
  const auto &B = gradient_operator(dN_global);
  return B.transpose() * C * B;
};

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_QUASIHARMONIC_INC_COMPUTATIONALDOMAIN_H_
