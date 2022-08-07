#include "../../inc/model/model.h"

namespace ffea {

Model::Model(Mesh& mesh, const Eigen::MatrixXd& constitutive_model,
             const DifferentialOperator& differential_operator,
             const std::vector<BoundaryCondition*>& boundary_conditions,
             ConditionFunction source)
    : mesh(mesh),
      constitutive_model(constitutive_model),
      differential_operator(differential_operator),
      boundary_conditions(boundary_conditions),
      source(source) {}

}  // namespace ffea
