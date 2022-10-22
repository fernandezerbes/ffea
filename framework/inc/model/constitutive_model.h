#ifndef FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_

#include <eigen3/Eigen/Dense>

#include "../geometry/coordinates.h"

namespace ffea {

class ConstitutiveModel {
 public:
  ConstitutiveModel(size_t n_cols, size_t n_rows);

  virtual Eigen::MatrixXd Evaluate(const Coordinates& coords) const = 0;

 protected:
  Eigen::MatrixXd constitutive_matrix_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_CONSTITUTIVEMODEL_H_
