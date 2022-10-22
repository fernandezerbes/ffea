#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_CONSTITUTIVEMODEL_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_CONSTITUTIVEMODEL_H_

#include "../../../framework/inc/model/constitutive_model.h"

namespace ffea {

class LinearElasticConstitutiveModel2D : public ConstitutiveModel {
 public:
  LinearElasticConstitutiveModel2D(double youngs_modulus, double poisson_ratio);

  virtual Matrix<double> Evaluate(const Coordinates& coords) const override;
};

class LinearElasticConstitutiveModel3D : public ConstitutiveModel {
 public:
  LinearElasticConstitutiveModel3D(double youngs_modulus, double poisson_ratio);

  virtual Matrix<double> Evaluate(const Coordinates& coords) const override;
};

}  // namespace ffea

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_CONSTITUTIVEMODEL_H_
