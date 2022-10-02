#include "../../inc/model/constitutive_model.h"

namespace ffea {

ConstitutiveModel::ConstitutiveModel(size_t n_cols, size_t n_rows)
    : constitutive_matrix_(Eigen::MatrixXd::Zero(n_cols, n_rows)) {}

}  // namespace ffea
