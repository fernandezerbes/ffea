#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_

#include <unordered_map>
#include <memory>
#include <vector>

#include "../math/shape_functions.h"
#include "../processor/operator.h"
#include "./element.h"
#include "./integration_point.h"
#include "./node.h"

namespace ffea {

enum class ElementType {
  kTwoNodeLine,
  kThreeNodeTria,
  kFourNodeQuad,
  kFourNodeTetra,
  kEightNodeHex,
  kSixNodePrism,
  kFiveNodePiramid,
  kThreeNodeLine,
  kSixNodeTria,
  kNineNodeQuad,
  kTenNodeTetra,
  kTwentySevenNodeHex,
  kEighteenNodePrism,
  kFourteenNodePiramid,
  kOneNodePoint,
  kEightNodeQuad,
  kTwentyNodeHex,
  kFifteenNodePrism,
  kThirteenNodePiramid
};

class ElementFactory {
 public:
  ElementFactory(ElementType element_type);

  Element CreateElement(const std::vector<Node *> &nodes) const;

 private:
  size_t dimension_;
  // TODO I don't like that the factory owns the shape functions and quadrature
  std::unique_ptr<ShapeFunctions> shape_functions_;
  std::unique_ptr<Quadrature> quadrature_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_
