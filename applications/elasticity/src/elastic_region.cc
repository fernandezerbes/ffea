#include "../inc/elastic_region.h"

#include "../inc/operator.h"

namespace ffea::app {

DynamicUndampedElasticDomain::DynamicUndampedElasticDomain(
    std::vector<Element>& elements, const DifferentialOperator B_operator,
    const ConstitutiveModel& constitutive_model, double density,
    SpatioTemporalFunction<std::vector<double>> source)
    : Domain(elements, B_operator, constitutive_model, source), density_(density) {}

void DynamicUndampedElasticDomain::Contribute(MassTerm& term, Element& element,
                                              size_t integration_point_idx, double t) const {}

// TODO: add DynamicDampedElasticDomain

}  // namespace ffea::app
