#include "../inc/constitutive_model.h"

namespace ffea {

IsotropicConductivityConstitutiveModel2D::
    IsotropicConductivityConstitutiveModel2D(double kxx, double kyy, double kxy)
    : ConstitutiveModel(2, 2) {
  constitutive_matrix_(0, 0) = -kxx;
  constitutive_matrix_(0, 1) = -kxy;
  constitutive_matrix_(1, 0) = -kxy;
  constitutive_matrix_(1, 1) = -kyy;
}

Eigen::MatrixXd IsotropicConductivityConstitutiveModel2D::Evaluate(
    const Coordinates& coords) const {
  return constitutive_matrix_;
}

}  // namespace ffea
