#include "../../inc/mesh/element_factory.h"

namespace ffea {

ElementFactory::ElementFactory(
    const IntegrationPointsProvider &integration_points_provider)
    : integration_points_provider_(integration_points_provider) {}

Element ElementFactory::CreateElement(
    GeometricEntity &geometric_entity,
    const std::vector<DegreeOfFreedom *> &dofs) const {
  auto geometric_entity_type = geometric_entity.type();
  const auto &integration_points =
      integration_points_provider_.GetIntegrationPoints(geometric_entity_type);
  return Element(geometric_entity, dofs, integration_points);
}

}  // namespace ffea
