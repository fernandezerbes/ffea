#include "../inc/constitutive_model.h"

namespace ffea {

LinearElasticConstitutiveModel2D::LinearElasticConstitutiveModel2D(double youngs_modulus,
                                                                   double poisson_ratio)
    : ConstitutiveModel(3, 3) {
  double factor = youngs_modulus / (1.0 - poisson_ratio * poisson_ratio);
  constitutive_matrix_(0, 0) = factor;
  constitutive_matrix_(0, 1) = factor * poisson_ratio;
  constitutive_matrix_(1, 0) = constitutive_matrix_(0, 1);
  constitutive_matrix_(1, 1) = factor;
  constitutive_matrix_(2, 2) = (1.0 - poisson_ratio) * factor;
}

Matrix<double> LinearElasticConstitutiveModel2D::Evaluate(const Coordinates& coords) const {
  return constitutive_matrix_;
}

LinearElasticConstitutiveModel3D::LinearElasticConstitutiveModel3D(double youngs_modulus,
                                                                   double poisson_ratio)
    : ConstitutiveModel(6, 6) {
  double factor = youngs_modulus / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
  constitutive_matrix_(0, 0) = factor * (1.0 - poisson_ratio);
  constitutive_matrix_(0, 1) = factor * poisson_ratio;
  constitutive_matrix_(0, 2) = factor * poisson_ratio;
  constitutive_matrix_(1, 1) = factor * (1.0 - poisson_ratio);
  constitutive_matrix_(1, 2) = poisson_ratio;
  constitutive_matrix_(2, 2) = factor * (1.0 - poisson_ratio);
  constitutive_matrix_(3, 3) = factor * (1.0 - 2.0 * poisson_ratio) / 2.0;
  constitutive_matrix_(4, 4) = factor * (1.0 - 2.0 * poisson_ratio) / 2.0;
  constitutive_matrix_(5, 5) = factor * (1.0 - 2.0 * poisson_ratio) / 2.0;
  constitutive_matrix_(1, 0) = constitutive_matrix_(0, 1);
  constitutive_matrix_(2, 0) = constitutive_matrix_(0, 2);
  constitutive_matrix_(2, 1) = constitutive_matrix_(1, 2);
}

Matrix<double> LinearElasticConstitutiveModel3D::Evaluate(const Coordinates& coords) const {
  return constitutive_matrix_;
}

}  // namespace ffea
