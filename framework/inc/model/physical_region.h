#ifndef FFEA_FRAMEWORK_MODEL_PHYSICALREGION_H_
#define FFEA_FRAMEWORK_MODEL_PHYSICALREGION_H_

#include <vector>

#include "../alias.h"
#include "../mesh/element.h"
#include "./equation.h"
#include "./operator.h"

namespace ffea {

class StiffnessTerm;
class MassTerm;
class DampingTerm;
class RhsTerm;

class PhysicalRegion {
 public:
  explicit PhysicalRegion(std::vector<Element>& elements);

  std::vector<Element>& elements() const;
  virtual void Contribute(StiffnessTerm& term, Element& element, size_t integration_point_idx,
                          Time t) const;
  virtual void Contribute(MassTerm& term, Element& element, size_t integration_point_idx,
                          Time t) const;
  virtual void Contribute(DampingTerm& term, Element& element, size_t integration_point_idx,
                          Time t) const;
  virtual void Contribute(RhsTerm& term, Element& element, size_t integration_point_idx,
                          Time t) const;

 protected:
  static void ComputeLoadContribution(Element& element,
                                      const SpatioTemporalFunction<std::vector<double>>& load,
                                      size_t integration_point_idx, Vector<double>& contribution,
                                      Time t);

 private:
  std::vector<Element>& elements_;
};

class Domain : public PhysicalRegion {
 public:
  Domain(std::vector<Element>& elements, DifferentialOperator differential_operator,
         const ConstitutiveModel& constitutive_model,
         SpatioTemporalFunction<std::vector<double>> source);

  void Contribute(StiffnessTerm& term, Element& element, size_t integration_point_idx,
                  Time t) const override;
  void Contribute(RhsTerm& term, Element& element, size_t integration_point_idx,
                  Time t) const override;

 protected:
  const DifferentialOperator differential_operator_;
  const ConstitutiveModel& constitutive_model_;
  const SpatioTemporalFunction<std::vector<double>> source_;
};

class DomainBoundary : public PhysicalRegion {
 public:
  DomainBoundary(std::vector<Element>& elements, SpatioTemporalFunction<std::vector<double>> load);

  void Contribute(RhsTerm& term, Element& element, size_t integration_point_idx,
                  Time t) const override;

 protected:
  const SpatioTemporalFunction<std::vector<double>> load_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_PHYSICALREGION_H_
