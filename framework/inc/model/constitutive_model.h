#ifndef FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_

#include <eigen3/Eigen/Dense>

#include "../geometry/coordinates.h"

namespace ffea {

class ConstitutiveModel {
 public:
  ConstitutiveModel(size_t n_cols, size_t n_rows);
  virtual ~ConstitutiveModel() = default;
  ConstitutiveModel(const ConstitutiveModel&) = default;
  ConstitutiveModel& operator=(const ConstitutiveModel&) = default;
  ConstitutiveModel(ConstitutiveModel&&) = default;
  ConstitutiveModel& operator=(ConstitutiveModel&&) = default;
  
  virtual Eigen::MatrixXd Evaluate(const Coordinates& coordinates) const = 0;

 protected:
  Eigen::MatrixXd constitutive_matrix_;
};

class LinearElasticConstitutiveModel2D : public ConstitutiveModel {
 public:
  LinearElasticConstitutiveModel2D(double youngs_modulus, double poisson_ratio);
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
};

class LinearElasticConstitutiveModel3D : public ConstitutiveModel {
 public:
  LinearElasticConstitutiveModel3D(double youngs_modulus, double poisson_ratio);
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
