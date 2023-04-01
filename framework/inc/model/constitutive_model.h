#ifndef FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_

#include "../alias.h"
#include "../geometry/coordinates.h"

namespace ffea {

class ConstitutiveModel {
 public:
  ConstitutiveModel(size_t n_rows, size_t n_cols);

  virtual Matrix<double> Evaluate(const Coordinates &coords) const = 0;

 protected:
  Matrix<double> constitutive_matrix_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
