#include "../inc/quasi_harmonic_region.h"

#include "../../../framework/inc/model/operator.h"

namespace ffea::app {

QuasiHarmonicDomainBoundary::QuasiHarmonicDomainBoundary(std::vector<Element>& elements,
                                                         VectorialFunction load,
                                                         ScalarFunction radiation)
    : DomainBoundary(elements, load), radiation_(radiation) {}

void QuasiHarmonicDomainBoundary::Contribute(StiffnessTerm& term, Element& element,
                                             size_t integration_point_idx) const {
  const auto& global_coords = element.global_coords(integration_point_idx);
  const auto& radiation_value = radiation_(global_coords);
  const auto weight = element.integration_point_weight(integration_point_idx);
  const auto differential = element.EvaluateDifferential(integration_point_idx);
  const auto& N = element.EvaluateShapeFunctions(integration_point_idx);
  const auto& contribution = N.transpose() * N * (radiation_value * weight * differential);
  term.element_matrix() += contribution;
}

}  // namespace ffea::app
