#include "../../inc/model/model.h"

namespace ffea {

Model::Model(Mesh& mesh, const Eigen::MatrixXd& constitutive_model,
             const DifferentialOperator& differential_operator,
             ConditionFunction source)
    : mesh_(mesh),
      constitutive_model(constitutive_model),
      differential_operator(differential_operator),
      source(source) {}

void Model::AddNeumannBoundaryCondition(const std::string& boundary_name,
                                        ConditionFunction boundary_function) {
  boundary_conditions_.push_back(
      std::make_unique<ffea::NeumannBoundaryCondition>(mesh_, boundary_name,
                                                       boundary_function));
}

void Model::AddDirichletBoundaryCondition(
    const std::string& boundary_name, ConditionFunction boundary_function,
    const std::unordered_set<size_t>& directions_to_consider,
    const EnforcementStrategy& enforcement_strategy) {
  boundary_conditions_.push_back(
      std::make_unique<ffea::DirichletBoundaryCondition>(
          mesh_, boundary_name, boundary_function, directions_to_consider,
          enforcement_strategy));
}

void Model::ProjectSolutionOnMesh(const Eigen::VectorXd& solution) {
  mesh_.SetSolutionOnDofs(solution);
}

}  // namespace ffea
