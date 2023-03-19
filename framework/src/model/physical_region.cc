#include "../../inc/model/physical_region.h"

namespace ffea {

PhysicalRegion::PhysicalRegion(std::vector<Element>& elements) : elements_(elements) {}

std::vector<Element>& PhysicalRegion::elements() const { return elements_; }

void PhysicalRegion::Contribute(StiffnessTerm& term, Element& element, size_t integration_point_idx,
                                double t) const {}

void PhysicalRegion::Contribute(MassTerm& term, Element& element, size_t integration_point_idx,
                                double t) const {}

void PhysicalRegion::Contribute(DampingTerm& term, Element& element, size_t integration_point_idx,
                                double t) const {}

void PhysicalRegion::Contribute(RhsTerm& term, Element& element, size_t integration_point_idx,
                                double t) const {}

void PhysicalRegion::ComputeLoadContribution(Element& element,
                                             SpatioTemporalFunction<std::vector<double>> load,
                                             size_t integration_point_idx,
                                             Vector<double>& contribution, double t) const {
  const auto& global_coords = element.global_coords(integration_point_idx);
  const auto& load_vector = load(global_coords, t);
  const auto weight = element.integration_point_weight(integration_point_idx);
  const auto differential = element.EvaluateDifferential(integration_point_idx);
  const auto number_of_nodes = element.number_of_nodes();

  const auto& N = element.EvaluateShapeFunctions(integration_point_idx);
  const auto number_of_components = load_vector.size();
  for (auto node_idx = 0; node_idx < number_of_nodes; node_idx++) {
    for (auto component_idx = 0; component_idx < number_of_components; component_idx++) {
      const auto& dof_idx = node_idx * number_of_components + component_idx;
      contribution(dof_idx) += N(0, node_idx) * load_vector[component_idx] * weight * differential;
    }
  }
}

Domain::Domain(std::vector<Element>& elements, const DifferentialOperator differential_operator,
               const ConstitutiveModel& constitutive_model,
               SpatioTemporalFunction<std::vector<double>> source)
    : PhysicalRegion(elements),
      differential_operator_(differential_operator),
      constitutive_model_(constitutive_model),
      source_(source) {}

void Domain::Contribute(StiffnessTerm& term, Element& element, size_t integration_point_idx,
                        double t) const {
  const auto& global_coords = element.global_coords(integration_point_idx);
  const auto& C = constitutive_model_.Evaluate(global_coords);
  const auto& dN_global = element.EvaluateGlobalShapeFunctionsDerivatives(integration_point_idx);
  const auto& operator_value = differential_operator_(dN_global);
  const auto& weight = element.integration_point_weight(integration_point_idx);
  const auto& differential = element.EvaluateDifferential(integration_point_idx);
  const auto& contribution =
      operator_value.transpose() * C * operator_value * weight * differential;
  term.element_matrix() += contribution;
}

void Domain::Contribute(RhsTerm& term, Element& element, size_t integration_point_idx,
                        double t) const {
  Vector<double> contribution = Vector<double>::Zero(element.number_of_dofs());
  ComputeLoadContribution(element, source_, integration_point_idx, contribution, t);
  term.element_vector() += contribution;
}

DomainBoundary::DomainBoundary(std::vector<Element>& elements,
                               SpatioTemporalFunction<std::vector<double>> load)
    : PhysicalRegion(elements), load_(load) {}

void DomainBoundary::Contribute(RhsTerm& term, Element& element, size_t integration_point_idx,
                                double t) const {
  Vector<double> contribution = Vector<double>::Zero(element.number_of_dofs());
  ComputeLoadContribution(element, load_, integration_point_idx, contribution, t);
  term.element_vector() += contribution;
}

}  // namespace ffea
