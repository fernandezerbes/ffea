#include "../../inc/geometry/geometric_entity_factory.h"

#include <stdexcept>

namespace ffea {
// TODO Change this to shared_ptr
std::shared_ptr<GeometricEntity>
GeometricEntityFactory2D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return std::make_shared<Line2D>(
          type, nodes, std::make_unique<TwoNodeLineShapeFunctions>());
    case GeometricEntityType::kThreeNodeTria:
      return std::make_shared<Tria2D>(
          type, nodes, std::make_unique<ThreeNodeTriaShapeFunctions>());
    case GeometricEntityType::kFourNodeQuad:
      return std::make_shared<Quad2D>(
          type, nodes, std::make_unique<FourNodeQuadShapeFunctions>());
    case GeometricEntityType::kSixNodeTria:
      return std::make_shared<Tria2D>(
          type, nodes, std::make_unique<SixNodeTriaShapeFunctions>());
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

std::shared_ptr<GeometricEntity>
GeometricEntityFactory3D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return std::make_shared<Line3D>(
          type, nodes, std::make_unique<TwoNodeLineShapeFunctions>());
    case GeometricEntityType::kThreeNodeTria:
      return std::make_shared<Tria3D>(
          type, nodes, std::make_unique<ThreeNodeTriaShapeFunctions>());
    case GeometricEntityType::kFourNodeQuad:
      return std::make_shared<Quad3D>(
          type, nodes, std::make_unique<FourNodeQuadShapeFunctions>());
    case GeometricEntityType::kFourNodeTetra:
      return std::make_shared<Tetra3D>(
          type, nodes, std::make_unique<FourNodeTetraShapeFunctions>());
    case GeometricEntityType::kEightNodeHex:
      return std::make_shared<Hex3D>(
          type, nodes, std::make_unique<EightNodeHexShapeFunctions>());
    case GeometricEntityType::kSixNodeTria:
      return std::make_shared<Tria3D>(
          type, nodes, std::make_unique<SixNodeTriaShapeFunctions>());
    case GeometricEntityType::kTenNodeTetra:
      return std::make_shared<Tetra3D>(
          type, nodes, std::make_unique<TenNodeTetraShapeFunctions>());
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

}  // namespace ffea
