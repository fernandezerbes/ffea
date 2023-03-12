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

using GeometricEntityGroups =
    std::unordered_map<std::string, std::vector<GeometricEntity *>>;

class Geometry {
 public:
  Geometry() = default;
  ~Geometry() = default;
  Geometry(const Geometry &) = delete;
  Geometry &operator=(const Geometry &) = delete;
  Geometry(Geometry &&) = default;
  Geometry &operator=(Geometry &&) = delete;

  size_t number_of_nodes() const;
  const std::vector<Node> &nodes() const;
  GeometricEntityGroups &entity_groups();
  std::vector<GeometricEntity *> &entity_group(const std::string &name);
  const std::vector<GeometricEntity *> &entity_group(
      const std::string &name) const;

  void AddNode(const std::array<double, 3> &xyz);
  void AddGeometricEntity(GeometricEntityType type,
                          const std::vector<size_t> &node_tags,
                          const GeometricEntityFactory &factory);
  void RegisterGeometricEntityGroup(const std::string &name,
                                    const std::vector<size_t> &entity_tags);

 private:
  std::vector<Node> nodes_;
  std::vector<std::unique_ptr<GeometricEntity>> entities_;
  GeometricEntityGroups entity_groups_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRY_H_
