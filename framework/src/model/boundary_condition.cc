#include "../../inc/model/boundary_condition.h"

namespace ffea {

// BoundaryCondition::BoundaryCondition(
//     const std::vector<Element> &boundary_elements,
//     ConditionFunction boundary_function)
//     : boundary_elements_(boundary_elements),
//       boundary_function_(boundary_function) {}

// NeumannBoundaryCondition::NeumannBoundaryCondition(
//     const std::vector<Element> &boundary_elements,
//     ConditionFunction boundary_function)
//     : BoundaryCondition(boundary_elements, boundary_function) {}

// void NeumannBoundaryCondition::Apply(Eigen::MatrixXd &stiffnes_matrix,
//                                      Eigen::VectorXd &rhs) {
//   for (auto& element : boundary_elements_) {
//     const auto& element_rhs = element.Co
//         element_processor.ProcessBoundaryElement(element, load_function);
//     const auto& dofs_map = element.GetLocalToGlobalDofIndicesMap();
//     // Scatter coefficients
//     for (size_t local_i_index = 0; local_i_index < dofs_map.size();
//          local_i_index++) {
//       size_t global_i_index = dofs_map[local_i_index];
//       global_rhs(global_i_index) += element_rhs(local_i_index);
//     }
//   }
// }

// class EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix, Eigen::VectorXd &rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) = 0;
// };

// class DirectEnforcementStrategy : public EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix, Eigen::VectorXd &rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) override;
// };

// class PenaltyEnforcementStrategy : public EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix, Eigen::VectorXd &rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) override;
// };

// class DirichletBoundaryCondition : public BoundaryCondition {
//  public:
//   DirichletBoundaryCondition(const std::vector<Element> &boundary_elements,
//                              ConditionFunction boundary_function,
//                              const std::vector<size_t>
//                              &directions_to_consider, const
//                              EnforcementStrategy &enforcement_strategy);

//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs) override;

//  private:
//   const std::vector<size_t> &directions_to_consider_;
//   const EnforcementStrategy &enforcement_strategy_;
// };

}  // namespace ffea