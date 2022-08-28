#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "../math/shape_functions.h"
#include "../processor/operator.h"
#include "./degree_of_freedom.h"
#include "./element.h"
#include "./integration_point.h"

namespace ffea {

class ElementFactory {
 public:
  ElementFactory(const Quadrature &quadrature);
  Element CreateElement(GeometricEntity &geometric_entity,
                        const std::vector<DegreeOfFreedom *> &dofs) const;

 private:
  const Quadrature &quadrature_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENTFACTORY_H_
