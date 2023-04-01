#include "../../inc/model/constitutive_model.h"

namespace ffea {

ConstitutiveModel::ConstitutiveModel(size_t n_rows, size_t n_cols)
    : constitutive_matrix_(Matrix<double>::Zero(n_cols, n_rows)) {}

}  // namespace ffea
