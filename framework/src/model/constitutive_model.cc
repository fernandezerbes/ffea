#include "../../inc/model/constitutive_model.h"

namespace ffea {

ConstitutiveModel::ConstitutiveModel(size_t n_rows, size_t n_cols)
    : constitutive_matrix_(Matrix<double>::Zero(n_rows, n_cols)) {}

}  // namespace ffea
