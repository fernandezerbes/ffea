#include "../../inc/mesh/integration_points_provider.h"

namespace ffea {

const IntegrationPointsGroup &IntegrationPointsProvider::GetIntegrationPoints(
    GeometricEntityType type) const {
  return integration_points_registry_.at(type);
}

void IntegrationPointsProvider::RegisterIntegrationPoints(
    GeometricEntityType type,
    const IntegrationPointsGroup &integration_points) {
  integration_points_registry_.insert({type, integration_points});
}

namespace utilities {

// TODO Review factories

IntegrationPointsProvider MakeFullIntegrationPointsProvider() {
  IntegrationPointsProvider provider;
  provider.RegisterIntegrationPoints(GeometricEntityType::kTwoNodeLine,
                                     rule_line_1);
  provider.RegisterIntegrationPoints(GeometricEntityType::kThreeNodeTria,
                                     rule_tria_1);
  provider.RegisterIntegrationPoints(GeometricEntityType::kFourNodeQuad,
                                     rule_quad_4);
  provider.RegisterIntegrationPoints(GeometricEntityType::kFourNodeTetra,
                                     rule_tetra_1);
  provider.RegisterIntegrationPoints(GeometricEntityType::kEightNodeHex,
                                     rule_hex_8);
  provider.RegisterIntegrationPoints(GeometricEntityType::kSixNodeTria,
                                     rule_tria_1);
  provider.RegisterIntegrationPoints(GeometricEntityType::kTenNodeTetra,
                                     rule_tetra_1);

  return provider;
}

IntegrationPointsProvider MakeReducedIntegrationPointsProvider() {
  IntegrationPointsProvider provider;
  provider.RegisterIntegrationPoints(GeometricEntityType::kTwoNodeLine,
                                     rule_line_2);
  provider.RegisterIntegrationPoints(GeometricEntityType::kThreeNodeTria,
                                     rule_tria_3);
  provider.RegisterIntegrationPoints(GeometricEntityType::kFourNodeQuad,
                                     rule_quad_4);
  provider.RegisterIntegrationPoints(GeometricEntityType::kFourNodeTetra,
                                     rule_tetra_1);
  provider.RegisterIntegrationPoints(GeometricEntityType::kEightNodeHex,
                                     rule_hex_8);
  provider.RegisterIntegrationPoints(GeometricEntityType::kSixNodeTria,
                                     rule_tria_1);
  provider.RegisterIntegrationPoints(GeometricEntityType::kTenNodeTetra,
                                     rule_tetra_1);
  return provider;
}

}  // namespace utilities

}  // namespace ffea
