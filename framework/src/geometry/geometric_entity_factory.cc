#include "../../inc/geometry/geometric_entity_factory.h"

#include <stdexcept>

namespace ffea {
// TODO Change this to shared_ptr
std::shared_ptr<GeometricEntity>
GeometricEntityFactory2D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return std::make_shared<TwoNodeLine2D>(nodes);
    case GeometricEntityType::kThreeNodeTria:
      return std::make_shared<ThreeNodeTria2D>(nodes);
    case GeometricEntityType::kFourNodeQuad:
      return std::make_shared<FourNodeQuad2D>(nodes);
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

std::shared_ptr<GeometricEntity>
GeometricEntityFactory3D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return std::make_shared<TwoNodeLine3D>(nodes);
    case GeometricEntityType::kThreeNodeTria:
      return std::make_shared<ThreeNodeTria3D>(nodes);
    case GeometricEntityType::kFourNodeQuad:
      return std::make_shared<FourNodeQuad3D>(nodes);
    case GeometricEntityType::kFourNodeTetra:
      return std::make_shared<FourNodeTetra3D>(nodes);
    case GeometricEntityType::kEightNodeHex:
      return std::make_shared<EightNodeHex3D>(nodes);
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

}  // namespace ffea
