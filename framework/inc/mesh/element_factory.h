#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_

#include <vector>

#include "./degree_of_freedom.h"
#include "./element.h"
#include "./integration_points_provider.h"

namespace ffea {

class ElementFactory {
 public:
  explicit ElementFactory(const IntegrationPointsProvider &ip_provider);

  Element CreateElement(GeometricEntity &entity, const std::vector<DegreeOfFreedom *> &dofs) const;

 private:
  const IntegrationPointsProvider &ip_provider_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_
