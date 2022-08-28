#ifndef FFEA_FRAMEWORK_INC_MESH_GEOMETRY_H_
#define FFEA_FRAMEWORK_INC_MESH_GEOMETRY_H_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "./coordinates.h"
#include "./geometric_entity.h"
#include "./geometric_entity_factory.h"
#include "./node.h"

namespace ffea {

class Geometry {
 public:
  Geometry();
  ~Geometry();

  size_t number_of_nodes() const;
  void AddNode(const std::array<double, 3>& xyz);
  void AddGeometricEntity(
      GeometricEntityType type, const std::string& group_name,
      const std::vector<size_t>& node_ids,
      const GeometricEntityFactory& geometric_entity_factory);
  std::vector<std::shared_ptr<GeometricEntity>>& GetGeometricEntityGroup(
      const std::string& group_name);
  const std::vector<std::shared_ptr<GeometricEntity>>& GetGeometricEntityGroup(
      const std::string& group_name) const;

  std::unordered_map<std::string, std::vector<std::shared_ptr<GeometricEntity>>>
      geometric_entities_groups_;
 private:
  std::vector<Node> nodes_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_GEOMETRY_H_
