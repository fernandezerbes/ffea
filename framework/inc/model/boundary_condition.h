#ifndef FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
#define FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_

#include <unordered_set>
#include <vector>

#include "../geometry/coordinates.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "../alias.h"

namespace ffea {

class BoundaryCondition {
 public:
  BoundaryCondition(const std::vector<Element> &elements);
  virtual ~BoundaryCondition() = default;
  BoundaryCondition(const BoundaryCondition &) = delete;
  BoundaryCondition &operator=(const BoundaryCondition &) = delete;
  BoundaryCondition(BoundaryCondition &&) = delete;
  BoundaryCondition &operator=(BoundaryCondition &&) = delete;

  virtual void Enforce(CSRMatrix<double> &global_stiffness,
                       Vector<double> &global_rhs) const = 0;

 protected:
  const std::vector<Element> &elements_;
};

class NaturalBoundaryCondition : public BoundaryCondition {
 public:
  NaturalBoundaryCondition(const std::vector<Element> &elements,
                           ConditionFunction load, ConditionFunction radiation);

  virtual void Enforce(CSRMatrix<double> &global_stiffness,
                       Vector<double> &global_rhs) const override;

 private:
  ConditionFunction load_;
  ConditionFunction radiation_;
};

class EnforcementStrategy {
 public:
  virtual void Enforce(CSRMatrix<double> &global_stiffness,
                       Vector<double> &global_rhs, ConditionFunction condition,
                       const std::vector<Element> &elements,
                       const std::unordered_set<size_t> &components) const = 0;
};

class DirectEnforcementStrategy : public EnforcementStrategy {
 public:
  virtual void Enforce(
      CSRMatrix<double> &global_stiffness, Vector<double> &global_rhs,
      ConditionFunction condition, const std::vector<Element> &elements,
      const std::unordered_set<size_t> &components) const override;
};

class PenaltyEnforcementStrategy : public EnforcementStrategy {
 public:
  PenaltyEnforcementStrategy(double penalty = 1.0e12);
  virtual void Enforce(
      CSRMatrix<double> &global_stiffness, Vector<double> &global_rhs,
      ConditionFunction condition, const std::vector<Element> &elements,
      const std::unordered_set<size_t> &components) const override;

 private:
  double penalty_;
};

class EssentialBoundaryCondition : public BoundaryCondition {
 public:
  EssentialBoundaryCondition(const std::vector<Element> &elements,
                             ConditionFunction condition,
                             const std::unordered_set<size_t> &components,
                             const EnforcementStrategy &strategy);

  virtual void Enforce(CSRMatrix<double> &global_stiffness,
                       Vector<double> &global_rhs) const override;

 private:
  ConditionFunction condition_;
  const std::unordered_set<size_t> &directions_to_consider_;
  const EnforcementStrategy &strategy_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
