#ifndef FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
#define FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include "../geometry/coordinates.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./types.h"
#include "./integrand.h"

namespace ffea {

class BoundaryCondition {
 public:
  BoundaryCondition(const std::vector<Element> &boundary_elements);
  virtual ~BoundaryCondition() = default;
  BoundaryCondition(const BoundaryCondition &) = delete;
  BoundaryCondition &operator=(const BoundaryCondition &) = delete;
  BoundaryCondition(BoundaryCondition &&) = default;
  BoundaryCondition &operator=(BoundaryCondition &&) = default;

  virtual void Enforce(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const = 0;

 protected:
  const std::vector<Element> &boundary_elements_;
};

class NeumannBoundaryCondition : public BoundaryCondition {
 public:
  NeumannBoundaryCondition(const std::vector<Element> &boundary_elements,
                           const PhysicsProcessor &processor);

  virtual void Enforce(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const override;

 private:
  const PhysicsProcessor &processor_;
};

class EnforcementStrategy {
 public:
  virtual void Enforce(
      Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
      ConditionFunction boundary_function,
      const std::vector<Element> &boundary_elements,
      const std::unordered_set<size_t> &directions_to_consider) const = 0;
};

class DirectEnforcementStrategy : public EnforcementStrategy {
 public:
  virtual void Enforce(
      Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
      ConditionFunction boundary_function,
      const std::vector<Element> &boundary_elements,
      const std::unordered_set<size_t> &directions_to_consider) const override;
};

class PenaltyEnforcementStrategy : public EnforcementStrategy {
 public:
  PenaltyEnforcementStrategy(double penalty = 1.0e12);
  virtual void Enforce(
      Eigen::MatrixXd &global_stiffness, Eigen::VectorXd &global_rhs,
      ConditionFunction boundary_function,
      const std::vector<Element> &boundary_elements,
      const std::unordered_set<size_t> &directions_to_consider) const override;

 private:
  double penalty_;
};

class DirichletBoundaryCondition : public BoundaryCondition {
 public:
  DirichletBoundaryCondition(
      const std::vector<Element> &boundary_elements,
      ConditionFunction boundary_function,
      const std::unordered_set<size_t> &directions_to_consider,
      const EnforcementStrategy &enforcement_strategy);

  virtual void Enforce(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const override;

 private:
  ConditionFunction boundary_function_;
  const std::unordered_set<size_t> &directions_to_consider_;
  const EnforcementStrategy &enforcement_strategy_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
