#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITYFACTORY_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITYFACTORY_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "../math/shape_functions.h"
#include "./geometric_entity.h"
#include "./node.h"

namespace ffea {

class GeometricEntityFactory {
 public:
  virtual std::unique_ptr<GeometricEntity> CreateGeometricEntity(
      GeometricEntityType type, const std::vector<Node *> &nodes) const = 0;
};

class GeometricEntityFactory2D : public GeometricEntityFactory {
 public:
  virtual std::unique_ptr<GeometricEntity> CreateGeometricEntity(
      GeometricEntityType type,
      const std::vector<Node *> &nodes) const override;
};

class GeometricEntityFactory3D : public GeometricEntityFactory {
 public:
  virtual std::unique_ptr<GeometricEntity> CreateGeometricEntity(
      GeometricEntityType type,
      const std::vector<Node *> &nodes) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITYFACTORY_H_
