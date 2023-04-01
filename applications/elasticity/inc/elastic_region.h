#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_ELASTICPHYSICALREGION_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_ELASTICPHYSICALREGION_H_

#include "../../../framework/inc/alias.h"
#include "../../../framework/inc/model/physical_region.h"

namespace ffea::app {

class DynamicUndampedElasticDomain : public Domain {
 public:
  DynamicUndampedElasticDomain(std::vector<Element>& elements,
                               const DifferentialOperator& B_operator,
                               const ConstitutiveModel& constitutive_model, double density,
                               SpatioTemporalFunction<std::vector<double>> source = nullptr);

  virtual void Contribute(MassTerm& term, Element& element, size_t integration_point_idx,
                          Time t) const override;

 protected:
  double density_;
};

// TODO: add DynamicDampedElasticDomain

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_ELASTICPHYSICALREGION_H_
