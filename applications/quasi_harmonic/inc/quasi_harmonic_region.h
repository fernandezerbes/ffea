#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_QUASIHARMONICREGION_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_QUASIHARMONICREGION_H_

#include "../../../framework/inc/alias.h"
#include "../../../framework/inc/model/operator.h"
#include "../../../framework/inc/model/physical_region.h"

namespace ffea::app {

class QuasiHarmonicDomainBoundary : DomainBoundary {
 public:
  QuasiHarmonicDomainBoundary(std::vector<Element>& elements,
                              SpatioTemporalFunction<std::vector<double>> load,
                              SpatioTemporalFunction<double> radiation);

  virtual void Contribute(StiffnessTerm& term, Element& element, size_t integration_point_idx,
                          double t) const override;

 private:
  const SpatioTemporalFunction<double> radiation_;
};

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_QUASIHARMONICREGION_H_
