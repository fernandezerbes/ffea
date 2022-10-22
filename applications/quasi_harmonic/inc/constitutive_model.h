#ifndef FFEA_APPLICATIONS_QUASIHARMONIC_INC_CONSTITUTIVEMODEL_H_
#define FFEA_APPLICATIONS_QUASIHARMONIC_INC_CONSTITUTIVEMODEL_H_

#include "../../../framework/inc/model/constitutive_model.h"

namespace ffea {

class IsotropicConductivityConstitutiveModel2D : public ConstitutiveModel {
 public:
  IsotropicConductivityConstitutiveModel2D(double k);

  virtual Eigen::MatrixXd Evaluate(const Coordinates& coords) const override;
};

class IsotropicConductivityConstitutiveModel3D : public ConstitutiveModel {
 public:
  IsotropicConductivityConstitutiveModel3D(double k);

  virtual Eigen::MatrixXd Evaluate(const Coordinates& coords) const override;
};

}  // namespace ffea

#endif  // FFEA_APPLICATIONS_QUASIHARMONIC_INC_CONSTITUTIVEMODEL_H_
