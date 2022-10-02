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

IsotropicConductivityConstitutiveModel3D::
    IsotropicConductivityConstitutiveModel3D(double kxx, double kyy, double kzz,
                                           double kxy, double kyz, double kxz)
    : ConstitutiveModel(3, 3) {
  constitutive_matrix_(0, 0) = -kxx;
  constitutive_matrix_(0, 1) = -kxy;
  constitutive_matrix_(0, 2) = -kxz;
  constitutive_matrix_(1, 0) = -kxy;
  constitutive_matrix_(1, 1) = -kyy;
  constitutive_matrix_(1, 2) = -kyz;
  constitutive_matrix_(2, 0) = -kxz;
  constitutive_matrix_(2, 1) = -kyz;
  constitutive_matrix_(2, 2) = -kzz;
}

Eigen::MatrixXd IsotropicConductivityConstitutiveModel3D::Evaluate(
    const Coordinates& coords) const {
  return constitutive_matrix_;
}

}  // namespace ffea
