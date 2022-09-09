#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINTSPROVIDER_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINTSPROVIDER_H_

#include "./integration_point.h"
#include "./geometric_entity.h"

namespace ffea {

class IntegrationPointsProvider {
 public:
  virtual const IntegrationPointsGroup &GetIntegrationPoints(
      GeometricEntityType type) const = 0;
};

class ReducedIntegrationPointsProvider : public IntegrationPointsProvider {
 public:
  virtual const IntegrationPointsGroup &GetIntegrationPoints(
      GeometricEntityType type) const override;
};

class FullIntegrationPointsProvider : public IntegrationPointsProvider {
 public:
  virtual const IntegrationPointsGroup &GetIntegrationPoints(
      GeometricEntityType type) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINTSPROVIDER_H_
