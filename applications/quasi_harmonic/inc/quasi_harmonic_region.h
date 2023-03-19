#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_QUASIHARMONICREGION_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_QUASIHARMONICREGION_H_

#include "../../../framework/inc/alias.h"
#include "../../../framework/inc/model/operator.h"
#include "../../../framework/inc/model/physical_region.h"

namespace ffea::app {

class QuasiHarmonicDomainBoundary : DomainBoundary {
 public:
  QuasiHarmonicDomainBoundary(std::vector<Element>& elements, VectorialFunction load,
                              ScalarFunction radiation);

  virtual void Contribute(StiffnessTerm& term, Element& element,
                          size_t integration_point_idx) const override;

 private:
  const ScalarFunction radiation_;
};

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_QUASIHARMONICREGION_H_
