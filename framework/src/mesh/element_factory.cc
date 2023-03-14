#include "../../inc/mesh/element_factory.h"

namespace ffea {

ElementFactory::ElementFactory(const IntegrationPointsTable &ip_table)
    : ip_table_(ip_table) {}

Element ElementFactory::CreateElement(GeometricEntity &entity,
                                      const std::vector<DegreeOfFreedom *> &dofs) const {
  const auto &ips = ip_table_[static_cast<int>(entity.type())];
  return Element(entity, dofs, ips);
}

}  // namespace ffea
