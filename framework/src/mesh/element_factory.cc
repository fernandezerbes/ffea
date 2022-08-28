#include "../../inc/mesh/element_factory.h"

namespace ffea {

ElementFactory::ElementFactory(const Quadrature &quadrature)
    : quadrature_(quadrature) {}

Element ElementFactory::CreateElement(
    GeometricEntity &geometric_entity,
    const std::vector<DegreeOfFreedom *> &dofs) const
{
  return Element(geometric_entity, dofs, quadrature_);
}

}  // namespace ffea
