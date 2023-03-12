#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINTSPROVIDER_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINTSPROVIDER_H_

#include <unordered_map>

#include "../geometry/geometric_entity.h"
#include "./integration_point.h"

namespace ffea {

class IntegrationPointsProvider {
 public:
  const IntegrationPointsGroup &integration_points(
      GeometricEntityType type) const;

  void RegisterIntegrationPoints(GeometricEntityType type,
                                 const IntegrationPointsGroup &ips);

 private:
  std::unordered_map<GeometricEntityType, const IntegrationPointsGroup &>
      ips_registry_;
};

namespace utilities {

IntegrationPointsProvider MakeFullIntegrationPointsProvider();

IntegrationPointsProvider MakeReducedIntegrationPointsProvider();

}  // namespace utilities

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINTSPROVIDER_H_
