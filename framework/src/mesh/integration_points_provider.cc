#include "../../inc/mesh/integration_points_provider.h"

namespace ffea {

const IntegrationPointsGroup&
ReducedIntegrationPointsProvider::GetIntegrationPoints(
    GeometricEntityType type) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return rule_line_1;
    case GeometricEntityType::kFourNodeQuad:
      return rule_quad_4; // TODO Check this rule
    case GeometricEntityType::kThreeNodeTria:
      return rule_tria_1;
    case GeometricEntityType::kFourNodeTetra:
      return rule_tetra_1;
    case GeometricEntityType::kEightNodeHex:
      return rule_hex_8; // TODO Check this rule
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

const IntegrationPointsGroup&
FullIntegrationPointsProvider::GetIntegrationPoints(
    GeometricEntityType type) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return rule_line_2;
    case GeometricEntityType::kFourNodeQuad:
      return rule_quad_4;
    case GeometricEntityType::kThreeNodeTria:
      return rule_tria_3;
    case GeometricEntityType::kFourNodeTetra:
      return rule_tetra_1; // TODO Check this rule
    case GeometricEntityType::kEightNodeHex:
      return rule_hex_8;
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

}  // namespace ffea