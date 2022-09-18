#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRY_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRY_H_

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

  // We need to provide a default move constructor because it's not implicitly
  // declared when having a user-declared destructor (~Geometry();)
  Geometry(Geometry&&) = default;

  size_t number_of_nodes() const;

  void AddNode(const std::array<double, 3>& xyz);

  void AddGeometricEntity(
      GeometricEntityType type, const std::vector<size_t>& node_ids,
      const GeometricEntityFactory& geometric_entity_factory);

  void RegisterGeometricEntityGroup(
      const std::string& group_name,
      const std::vector<size_t>& geometric_entities_ids);

  std::vector<GeometricEntity*>& GetGeometricEntityGroup(
      const std::string& group_name);

  const std::vector<GeometricEntity*>& GetGeometricEntityGroup(
      const std::string& group_name) const;

  const std::vector<Node>& nodes() const;

 private:
  std::vector<Node> nodes_;
  std::vector<std::unique_ptr<GeometricEntity>> geometric_entities_;

 public:
  // TODO Make private
  std::unordered_map<std::string, std::vector<GeometricEntity*>>
      geometric_entities_groups_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRY_H_
