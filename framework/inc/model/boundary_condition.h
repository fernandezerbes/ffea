#ifndef FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
#define FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_

#include <unordered_set>
#include <vector>

#include "../alias.h"
#include "../geometry/coordinates.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"

namespace ffea {

class EnforcementStrategy;

class EssentialBoundaryCondition {
 public:
  EssentialBoundaryCondition(std::vector<Element> &elements,
                             SpatioTemporalFunction<std::vector<double>> condition,
                             const std::unordered_set<size_t> &components,
                             EnforcementStrategy &strategy);

  void Process(CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs, double t);

 private:
  std::vector<Element> &elements_;
  SpatioTemporalFunction<std::vector<double>> condition_;
  const std::unordered_set<size_t> &directions_to_consider_;
  EnforcementStrategy &strategy_;
};

class EnforcementStrategy {
 public:
  virtual void Enforce(CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs,
                       SpatioTemporalFunction<std::vector<double>> condition,
                       std::vector<Element> &elements, const std::unordered_set<size_t> &components,
                       double t) = 0;
};

class DirectEnforcementStrategy : public EnforcementStrategy {
 public:
  virtual void Enforce(CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs,
                       SpatioTemporalFunction<std::vector<double>> condition,
                       std::vector<Element> &elements, const std::unordered_set<size_t> &components,
                       double t) override;
};

class PenaltyEnforcementStrategy : public EnforcementStrategy {
 public:
  PenaltyEnforcementStrategy(double penalty = 1.0e12);
  virtual void Enforce(CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs,
                       SpatioTemporalFunction<std::vector<double>> condition,
                       std::vector<Element> &elements, const std::unordered_set<size_t> &components,
                       double t) override;

 private:
  double penalty_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
