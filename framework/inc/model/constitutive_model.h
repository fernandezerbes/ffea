#ifndef FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_

#include <eigen3/Eigen/Dense>

#include "../mesh/coordinates.h"

namespace ffea {

class ConstitutiveModel {
public:
 virtual ~ConstitutiveModel();
 virtual Eigen::MatrixXd Evaluate(const Coordinates& coordinates) const = 0;
};

class LinearElasticConstitutiveModel2D : public ConstitutiveModel {
public:
 LinearElasticConstitutiveModel2D(double youngs_modulus, double poisson_ratio);
 virtual ~LinearElasticConstitutiveModel2D();
 virtual Eigen::MatrixXd Evaluate(const Coordinates& coordinates) const override;

private:
 Eigen::MatrixXd constitutive_matrix_;
};

class LinearElasticConstitutiveModel3D : public ConstitutiveModel {
public:
 LinearElasticConstitutiveModel3D(double youngs_modulus, double poisson_ratio);
 virtual ~LinearElasticConstitutiveModel3D();
 virtual Eigen::MatrixXd Evaluate(const Coordinates& coordinates) const override;

private:
 Eigen::MatrixXd constitutive_matrix_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
