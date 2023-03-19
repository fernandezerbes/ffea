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
  virtual void Contribute(StiffnessTerm& term, Element& element,
                          size_t integration_point_idx) const;
  virtual void Contribute(MassTerm& term, Element& element, size_t integration_point_idx) const;
  virtual void Contribute(DampingTerm& term, Element& element, size_t integration_point_idx) const;
  virtual void Contribute(RhsTerm& term, Element& element, size_t integration_point_idx) const;

 protected:
  void ComputeLoadContribution(Element& element, VectorialFunction load,
                               size_t integration_point_idx, Vector<double>& contribution) const;

 private:
  std::vector<Element>& elements_;
};

class Domain : public PhysicalRegion {
 public:
  Domain(std::vector<Element>& elements, const DifferentialOperator differential_operator,
         const ConstitutiveModel& constitutive_model, VectorialFunction source);

  virtual void Contribute(StiffnessTerm& term, Element& element,
                          size_t integration_point_idx) const override;
  virtual void Contribute(RhsTerm& term, Element& element,
                          size_t integration_point_idx) const override;

 protected:
  const DifferentialOperator differential_operator_;
  const ConstitutiveModel& constitutive_model_;
  const VectorialFunction source_;
};

class DomainBoundary : public PhysicalRegion {
 public:
  DomainBoundary(std::vector<Element>& elements, VectorialFunction load);

  virtual void Contribute(RhsTerm& term, Element& element,
                          size_t integration_point_idx) const override;

 protected:
  const VectorialFunction load_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_PHYSICALREGION_H_
