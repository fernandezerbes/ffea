#ifndef FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
#define FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <string>
#include <vector>

#include "../mesh/coordinates.h"
#include "../mesh/element.h"

namespace ffea {

// class BoundaryCondition {
//  public:
//   BoundaryCondition(const std::vector<Element> &boundary_elements,
//                     ConditionFunction boundary_function);
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs) = 0;

//  protected:
//   const std::vector<Element> &boundary_elements_;
//   const ConditionFunction boundary_function_;
// };

// class NeumannBoundaryCondition : public BoundaryCondition {
//  public:
//   NeumannBoundaryCondition(const std::vector<Element> &boundary_elements,
//                            ConditionFunction boundary_function);

//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs) override;
// };

// class EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) = 0;
// };

// class DirectEnforcementStrategy : public EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) override;
// };

// class PenaltyEnforcementStrategy : public EnforcementStrategy {
//  public:
//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs,
//                      ConditionFunction boundary_function,
//                      const std::vector<Element> &boundary_elements) override;
// };

// class DirichletBoundaryCondition : public BoundaryCondition {
//  public:
//   DirichletBoundaryCondition(const std::vector<Element> &boundary_elements,
//                              ConditionFunction boundary_function,
//                              const std::vector<size_t> &directions_to_consider,
//                              const EnforcementStrategy &enforcement_strategy);

//   virtual void Apply(Eigen::MatrixXd &stiffnes_matrix,
//                      Eigen::VectorXd &rhs) override;

//  private:
//   const std::vector<size_t> &directions_to_consider_;
//   const EnforcementStrategy &enforcement_strategy_;
// };

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_