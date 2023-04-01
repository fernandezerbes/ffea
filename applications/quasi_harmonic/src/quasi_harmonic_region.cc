#include "../inc/quasi_harmonic_region.h"

#include <utility>

#include "../../../framework/inc/model/operator.h"

namespace ffea::app {

QuasiHarmonicDomainBoundary::QuasiHarmonicDomainBoundary(
    std::vector<Element>& elements, SpatioTemporalFunction<std::vector<double>> load,
    SpatioTemporalFunction<double> radiation)
    : DomainBoundary(elements, std::move(load)), radiation_(std::move(radiation)) {}

void QuasiHarmonicDomainBoundary::Contribute(StiffnessTerm& term, Element& element,
                                             size_t integration_point_idx, Time t) const {
  const auto& global_coords = element.global_coords(integration_point_idx);
  const auto& radiation_value = radiation_(global_coords, t);
  const auto weight = element.integration_point_weight(integration_point_idx);
  const auto differential = element.EvaluateDifferential(integration_point_idx);
  const auto& N = element.EvaluateShapeFunctions(integration_point_idx);
  const auto& contribution = N.transpose() * N * (radiation_value * weight * differential);
  term.element_matrix() += contribution;
}

}  // namespace ffea::app
