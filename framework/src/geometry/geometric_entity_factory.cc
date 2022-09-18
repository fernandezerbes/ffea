#include "../../inc/geometry/geometric_entity_factory.h"

#include <stdexcept>

namespace ffea {

std::unique_ptr<GeometricEntity>
GeometricEntityFactory2D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return std::make_unique<Line2D>(
          type, nodes, std::make_unique<TwoNodeLineShapeFunctions>());
    case GeometricEntityType::kThreeNodeTria:
      return std::make_unique<Tria2D>(
          type, nodes, std::make_unique<ThreeNodeTriaShapeFunctions>());
    case GeometricEntityType::kFourNodeQuad:
      return std::make_unique<Quad2D>(
          type, nodes, std::make_unique<FourNodeQuadShapeFunctions>());
    case GeometricEntityType::kSixNodeTria:
      return std::make_unique<Tria2D>(
          type, nodes, std::make_unique<SixNodeTriaShapeFunctions>());
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

std::unique_ptr<GeometricEntity>
GeometricEntityFactory3D::CreateGeometricEntity(
    GeometricEntityType type, const std::vector<Node *> &nodes) const {
  switch (type) {
    case GeometricEntityType::kTwoNodeLine:
      return std::make_unique<Line3D>(
          type, nodes, std::make_unique<TwoNodeLineShapeFunctions>());
    case GeometricEntityType::kThreeNodeTria:
      return std::make_unique<Tria3D>(
          type, nodes, std::make_unique<ThreeNodeTriaShapeFunctions>());
    case GeometricEntityType::kFourNodeQuad:
      return std::make_unique<Quad3D>(
          type, nodes, std::make_unique<FourNodeQuadShapeFunctions>());
    case GeometricEntityType::kFourNodeTetra:
      return std::make_unique<Tetra3D>(
          type, nodes, std::make_unique<FourNodeTetraShapeFunctions>());
    case GeometricEntityType::kEightNodeHex:
      return std::make_unique<Hex3D>(
          type, nodes, std::make_unique<EightNodeHexShapeFunctions>());
    case GeometricEntityType::kSixNodeTria:
      return std::make_unique<Tria3D>(
          type, nodes, std::make_unique<SixNodeTriaShapeFunctions>());
    case GeometricEntityType::kTenNodeTetra:
      return std::make_unique<Tetra3D>(
          type, nodes, std::make_unique<TenNodeTetraShapeFunctions>());
    default:
      throw std::runtime_error("Unsupported GeometricEntityType");
  }
}

}  // namespace ffea
