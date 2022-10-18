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
  Geometry() = default;

  // We need the move-constructor and move-assignment operator defined to be
  // able to move the vector of unique_ptr. The rest is just rule of 5.
  ~Geometry() = default;
  Geometry(const Geometry&) = delete;
  Geometry& operator=(const Geometry&) = delete;
  Geometry(Geometry&&) = default;
  Geometry& operator=(Geometry&&) = delete;

  size_t number_of_nodes() const;

  void AddNode(const std::array<double, 3>& xyz);

  void AddGeometricEntity(
      GeometricEntityType type, const std::vector<size_t>& node_tags,
      const GeometricEntityFactory& geometric_entity_factory);

  void RegisterGeometricEntityGroup(
      const std::string& group_name,
      const std::vector<size_t>& geometric_entity_tags);

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
