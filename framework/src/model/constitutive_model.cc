#include "../../inc/model/constitutive_model.h"

namespace ffea {

ConstitutiveModel::ConstitutiveModel(size_t n_cols, size_t n_rows)
    : constitutive_matrix_(Eigen::MatrixXd::Zero(n_cols, n_rows)) {}

LinearElasticConstitutiveModel2D::LinearElasticConstitutiveModel2D(
    double youngs_modulus, double poisson_ratio)
    : ConstitutiveModel(3, 3) {
  double factor = youngs_modulus / (1.0 - poisson_ratio * poisson_ratio);
  constitutive_matrix_(0, 0) = factor;
  constitutive_matrix_(0, 1) = factor * poisson_ratio;
  constitutive_matrix_(1, 0) = constitutive_matrix_(0, 1);
  constitutive_matrix_(1, 1) = factor;
  constitutive_matrix_(2, 2) = (1.0 - poisson_ratio) * factor;
}

Eigen::MatrixXd LinearElasticConstitutiveModel2D::Evaluate(
    const Coordinates& coords) const {
  return constitutive_matrix_;
}

LinearElasticConstitutiveModel3D::LinearElasticConstitutiveModel3D(
    double youngs_modulus, double poisson_ratio)
    : ConstitutiveModel(6, 6) {
  double factor =
      youngs_modulus / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
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

Eigen::MatrixXd LinearElasticConstitutiveModel3D::Evaluate(
    const Coordinates& coords) const {
  return constitutive_matrix_;
}

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
