#include "../../inc/mesh/element_factory.h"

namespace ffea {

ElementFactory::ElementFactory(const IntegrationPointsProvider &ip_provider)
    : ip_provider_(ip_provider) {}

Element ElementFactory::CreateElement(GeometricEntity &entity,
                                      const std::vector<DegreeOfFreedom *> &dofs) const {
  auto type = entity.type();
  const auto &ips = ip_provider_.integration_points(type);
  return Element(entity, dofs, ips);
}

}  // namespace ffea
