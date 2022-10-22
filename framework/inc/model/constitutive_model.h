#ifndef FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_

#include "../geometry/coordinates.h"
#include "../alias.h"

namespace ffea {

class ConstitutiveModel {
 public:
  ConstitutiveModel(size_t n_cols, size_t n_rows);

  virtual Matrix<double> Evaluate(const Coordinates& coords) const = 0;

 protected:
  Matrix<double> constitutive_matrix_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
