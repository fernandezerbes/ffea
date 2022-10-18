#include "../../inc/geometry/geometry.h"

#include <stdexcept>
namespace ffea {

std::vector<GeometricEntity*>& Geometry::GetGeometricEntityGroup(
    const std::string& group_name) {
  return geometric_entities_groups_.at(group_name);
}

const std::vector<GeometricEntity*>& Geometry::GetGeometricEntityGroup(
    const std::string& group_name) const {
  return geometric_entities_groups_.at(group_name);
}

void Geometry::AddNode(const std::array<double, 3>& xyz) {
  size_t tag = number_of_nodes();
  nodes_.emplace_back(tag, xyz);
}

void Geometry::AddGeometricEntity(GeometricEntityType type,
                                  const std::vector<size_t>& node_tags,
                                  const GeometricEntityFactory& factory) {
  std::vector<Node*> nodes;
  nodes.reserve(node_tags.size());
  for (size_t tag : node_tags) {
    nodes.push_back(&nodes_[tag]);
  }

  auto geometric_entity = factory.CreateGeometricEntity(type, nodes);
  geometric_entities_.push_back(std::move(geometric_entity));
}

void Geometry::RegisterGeometricEntityGroup(
    const std::string& group_name,
    const std::vector<size_t>& geometric_entity_tags) {
  if (geometric_entities_groups_.contains(group_name)) {
    throw std::runtime_error("Entity group already registered");
  };

  std::vector<GeometricEntity*> geometric_entities;
  geometric_entities.reserve(geometric_entity_tags.size());
  for (auto tag : geometric_entity_tags) {
    geometric_entities.push_back(geometric_entities_[tag].get());
  }

  geometric_entities_groups_.insert({group_name, geometric_entities});
}

size_t Geometry::number_of_nodes() const { return nodes_.size(); }

const std::vector<Node>& Geometry::nodes() const { return nodes_; }

}  // namespace ffea
