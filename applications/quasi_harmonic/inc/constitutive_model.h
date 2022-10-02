#ifndef FFEA_APPLICATIONS_QUASIHARMONIC_INC_CONSTITUTIVEMODEL_H_
#define FFEA_APPLICATIONS_QUASIHARMONIC_INC_CONSTITUTIVEMODEL_H_

#include <eigen3/Eigen/Dense>

#include "../../../framework/inc/model/constitutive_model.h"

namespace ffea {

class IsotropicConductivityConstitutiveModel3D : public ConstitutiveModel {
 public:
  IsotropicConductivityConstitutiveModel3D(double kxx, double kyy, double kzz,
                                           double kxy, double kyz, double kxz);

  virtual Eigen::MatrixXd Evaluate(const Coordinates& coords) const override;
};

class IsotropicConductivityConstitutiveModel2D : public ConstitutiveModel {
 public:
  IsotropicConductivityConstitutiveModel2D(double kxx, double kyy, double kxy);

  virtual Eigen::MatrixXd Evaluate(const Coordinates& coords) const override;
};

}  // namespace ffea

#endif  // FFEA_APPLICATIONS_QUASIHARMONIC_INC_CONSTITUTIVEMODEL_H_
