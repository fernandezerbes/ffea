#include "../../inc/geometry/geometric_entity_factory.h"

#include <stdexcept>

namespace ffea {

std::unique_ptr<GeometricEntity> GeometricEntityFactory2D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kOneNodePoint:
      return std::make_unique<Point>(2, nodes);
    case GeometricEntityType::kTwoNodeLine:
      return std::make_unique<TwoNodeLine>(2, nodes);
    case GeometricEntityType::kThreeNodeTria:
      return std::make_unique<ThreeNodeTria>(2, nodes);
    case GeometricEntityType::kFourNodeQuad:
      return std::make_unique<FourNodeQuad>(2, nodes);
    case GeometricEntityType::kSixNodeTria:
      return std::make_unique<SixNodeTria>(2, nodes);
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

std::unique_ptr<GeometricEntity> GeometricEntityFactory3D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kOneNodePoint:
      return std::make_unique<Point>(3, nodes);
    case GeometricEntityType::kTwoNodeLine:
      return std::make_unique<TwoNodeLine>(3, nodes);
    case GeometricEntityType::kThreeNodeTria:
      return std::make_unique<ThreeNodeTria>(3, nodes);
    case GeometricEntityType::kFourNodeQuad:
      return std::make_unique<FourNodeQuad>(3, nodes);
    case GeometricEntityType::kFourNodeTetra:
      return std::make_unique<FourNodeTetra>(nodes);
    case GeometricEntityType::kEightNodeHex:
      return std::make_unique<EightNodeHex>(nodes);
    case GeometricEntityType::kSixNodeTria:
      return std::make_unique<SixNodeTria>(3, nodes);
    case GeometricEntityType::kTenNodeTetra:
      return std::make_unique<TenNodeTetra>(nodes);
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

}  // namespace ffea
