#include "../../inc/model/boundary_condition.h"

namespace ffea {

BoundaryCondition::BoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function)
    : boundary_elements_(boundary_elements),
      boundary_function_(boundary_function) {}

NeumannBoundaryCondition::NeumannBoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function)
    : BoundaryCondition(boundary_elements, boundary_function) {}

void NeumannBoundaryCondition::Apply(Eigen::MatrixXd &global_stiffness,
                                     Eigen::VectorXd &global_rhs) const {
  for (auto &element : boundary_elements_) {
    const auto &element_rhs = element.ComputeRhs(boundary_function_);
    const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
    // Scatter coefficients
    for (size_t local_i_index = 0; local_i_index < dofs_map.size();
         local_i_index++) {
      size_t global_i_index = dofs_map[local_i_index];
      global_rhs(global_i_index) += element_rhs(local_i_index);
    }
  }
}

// class DirectEnforcementStrategy : public EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &global_stiffness, Eigen::VectorXd
//   &global_rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) override;
// };

PenaltyEnforcementStrategy::PenaltyEnforcementStrategy(double penalty)
    : penalty_(penalty) {}

void PenaltyEnforcementStrategy::Apply(
    Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
    ConditionFunction boundary_function,
    const std::vector<Element> &boundary_elements) const {
  double boundary_value = 0.0;  // TODO This should be changed to accept a
                                // lambda with the components

  for (auto &element : boundary_elements) {
    const auto &dofs_map = element.GetLocalToGlobalDofIndicesMap();
    for (const auto &global_i_index : dofs_map) {
      global_stiffness.coeffRef(global_i_index, global_i_index) *= penalty_;
      global_rhs(global_i_index) = boundary_value * penalty_;
    }
  }
}

DirichletBoundaryCondition::DirichletBoundaryCondition(
    const std::vector<Element> &boundary_elements,
    ConditionFunction boundary_function,
    const std::vector<size_t> &directions_to_consider,
    const EnforcementStrategy &enforcement_strategy)
    : BoundaryCondition(boundary_elements, boundary_function),
      directions_to_consider_(directions_to_consider),
      enforcement_strategy_(enforcement_strategy) {}

void DirichletBoundaryCondition::Apply(Eigen::MatrixXd &global_stiffness,
                                       Eigen::VectorXd &global_rhs) const {
  enforcement_strategy_.Apply(global_stiffness, global_rhs, boundary_function_,
                              boundary_elements_);
}

}  // namespace ffea