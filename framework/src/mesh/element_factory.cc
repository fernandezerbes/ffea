#include "../../inc/mesh/element_factory.h"

namespace ffea {

ElementFactory::ElementFactory(ElementType element_type)
    : shape_functions_(), quadrature_() {
  switch (element_type) {
    case ElementType::kTwoNodeLine:
      dimension_ = 1;
      shape_functions_ = std::make_unique<Linear1DShapeFunctions>();
      quadrature_ = std::make_unique<QuadratureRule1x2>();
      break;
    case ElementType::kThreeNodeTria:
      break;
    case ElementType::kFourNodeQuad:
      dimension_ = 2;
      shape_functions_ = std::make_unique<Linear2DShapeFunctions>();
      quadrature_ = std::make_unique<QuadratureRule2x2>();
      break;
    case ElementType::kFourNodeTetra:
      break;
    case ElementType::kEightNodeHex:
      break;
    case ElementType::kSixNodePrism:
      break;
    case ElementType::kFiveNodePiramid:
      break;
    case ElementType::kThreeNodeLine:
      break;
    case ElementType::kSixNodeTria:
      break;
    case ElementType::kNineNodeQuad:
      break;
    case ElementType::kTenNodeTetra:
      break;
    case ElementType::kTwentySevenNodeHex:
      break;
    case ElementType::kEighteenNodePrism:
      break;
    case ElementType::kFourteenNodePiramid:
      break;
    case ElementType::kOneNodePoint:
      break;
    case ElementType::kEightNodeQuad:
      break;
    case ElementType::kTwentyNodeHex:
      break;
    case ElementType::kFifteenNodePrism:
      break;
    case ElementType::kThirteenNodePiramid:
      break;
    default:
      break;
  }
}

Element ElementFactory::CreateElement(const std::vector<Node *> &nodes) const {
  return Element(
      dimension_, nodes, *shape_functions_,
      quadrature_->GetIntegrationPoints()); 
      // TODO See how to modify the integration points, maybe with a factory
}

}  // namespace ffea
