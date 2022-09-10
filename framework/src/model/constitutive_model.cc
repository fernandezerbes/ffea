#include "../../inc/model/constitutive_model.h"

namespace ffea {

ConstitutiveModel::~ConstitutiveModel() {}

LinearElasticConstitutiveModel2D::LinearElasticConstitutiveModel2D(
    double youngs_modulus, double poisson_ratio)
    : constitutive_matrix_(Eigen::MatrixXd::Zero(3, 3)) {
  double factor = youngs_modulus / (1.0 - poisson_ratio * poisson_ratio);
  constitutive_matrix_(0, 0) = factor;
  constitutive_matrix_(0, 1) = factor * poisson_ratio;
  constitutive_matrix_(1, 0) = constitutive_matrix_(0, 1);
  constitutive_matrix_(1, 1) = factor;
  constitutive_matrix_(2, 2) = (1.0 - poisson_ratio) * factor;
}

LinearElasticConstitutiveModel2D::~LinearElasticConstitutiveModel2D() {}

Eigen::MatrixXd LinearElasticConstitutiveModel2D::Evaluate(
    const Coordinates& coordinates) const {
  return constitutive_matrix_;
}

LinearElasticConstitutiveModel3D::LinearElasticConstitutiveModel3D(
    double youngs_modulus, double poisson_ratio)
    : constitutive_matrix_(Eigen::MatrixXd::Zero(6, 6)) {
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

LinearElasticConstitutiveModel3D::~LinearElasticConstitutiveModel3D() {}

Eigen::MatrixXd LinearElasticConstitutiveModel3D::Evaluate(
    const Coordinates& coordinates) const {
  return constitutive_matrix_;
}

}  // namespace ffea
