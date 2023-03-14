#include "../inc/constitutive_model.h"

namespace ffea::app {

IsotropicConductivityConstitutiveModel2D::IsotropicConductivityConstitutiveModel2D(double k)
    : ConstitutiveModel(2, 2) {
  constitutive_matrix_(0, 0) = k;
  constitutive_matrix_(1, 1) = k;
}

Matrix<double> IsotropicConductivityConstitutiveModel2D::Evaluate(const Coordinates& coords) const {
  return constitutive_matrix_;
}

IsotropicConductivityConstitutiveModel3D::IsotropicConductivityConstitutiveModel3D(double k)
    : ConstitutiveModel(3, 3) {
  constitutive_matrix_(0, 0) = k;
  constitutive_matrix_(1, 1) = k;
  constitutive_matrix_(2, 2) = k;
}

Matrix<double> IsotropicConductivityConstitutiveModel3D::Evaluate(const Coordinates& coords) const {
  return constitutive_matrix_;
}

}  // namespace ffea::app
