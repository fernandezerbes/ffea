#ifndef FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
#define FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_

#include <unordered_set>
#include <vector>

#include "../alias.h"
#include "../geometry/coordinates.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./physics_processor.h"

namespace ffea {

class NaturalBoundaryCondition : public PhysicsProcessor {
 public:
  NaturalBoundaryCondition(const std::vector<Element> &elements,
                           VectorialFunction load, VectorialFunction radiation);

  virtual void Process(CSRMatrix<double> &system_stiffness,
                       Vector<double> &system_rhs) const override;
  virtual void Process(CSRMatrix<double> &system_stiffness,
                       CSRMatrix<double> &system_mass,
                       Vector<double> &system_rhs) const override;
  virtual void Process(CSRMatrix<double> &system_stiffness,
                       CSRMatrix<double> &system_mass,
                       CSRMatrix<double> &system_damping,
                       Vector<double> &system_rhs) const override;

 private:
  VectorialFunction load_;
  VectorialFunction radiation_;
};

class EnforcementStrategy {
 public:
  virtual void Enforce(CSRMatrix<double> &system_stiffness,
                       Vector<double> &system_rhs, VectorialFunction condition,
                       const std::vector<Element> &elements,
                       const std::unordered_set<size_t> &components) const = 0;
};

class DirectEnforcementStrategy : public EnforcementStrategy {
 public:
  virtual void Enforce(
      CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs,
      VectorialFunction condition, const std::vector<Element> &elements,
      const std::unordered_set<size_t> &components) const override;
};

class PenaltyEnforcementStrategy : public EnforcementStrategy {
 public:
  PenaltyEnforcementStrategy(double penalty = 1.0e12);
  virtual void Enforce(
      CSRMatrix<double> &system_stiffness, Vector<double> &system_rhs,
      VectorialFunction condition, const std::vector<Element> &elements,
      const std::unordered_set<size_t> &components) const override;

 private:
  double penalty_;
};

class EssentialBoundaryCondition : public PhysicsProcessor {
 public:
  EssentialBoundaryCondition(const std::vector<Element> &elements,
                             VectorialFunction condition,
                             const std::unordered_set<size_t> &components,
                             const EnforcementStrategy &strategy);

  virtual void Process(CSRMatrix<double> &system_stiffness,
                       Vector<double> &system_rhs) const override;
  virtual void Process(CSRMatrix<double> &system_stiffness,
                       CSRMatrix<double> &system_mass,
                       Vector<double> &system_rhs) const override;
  virtual void Process(CSRMatrix<double> &system_stiffness,
                       CSRMatrix<double> &system_mass,
                       CSRMatrix<double> &system_damping,
                       Vector<double> &system_rhs) const override;

 private:
  VectorialFunction condition_;
  const std::unordered_set<size_t> &directions_to_consider_;
  const EnforcementStrategy &strategy_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_BOUNDARYCONDITION_H_
