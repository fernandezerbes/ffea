#include "../../inc/geometry/geometry.h"

#include <stdexcept>

namespace ffea {

size_t Geometry::number_of_nodes() const { return nodes_.size(); }

const std::vector<Node>& Geometry::nodes() const { return nodes_; }

GeometricEntityGroups& Geometry::entity_groups() { return entity_groups_; }

std::vector<GeometricEntity*>& Geometry::entity_group(const std::string& name) {
  return entity_groups_.at(name);
}

const std::vector<GeometricEntity*>& Geometry::entity_group(const std::string& name) const {
  return entity_groups_.at(name);
}

void Geometry::AddNode(const std::array<double, 3>& xyz) {
  size_t tag = number_of_nodes();
  nodes_.emplace_back(tag, xyz);
}

void Geometry::AddGeometricEntity(GeometricEntityType type, const std::vector<size_t>& node_tags,
                                  const GeometricEntityFactory& factory) {
  std::vector<Node*> nodes;
  nodes.reserve(node_tags.size());
  for (size_t tag : node_tags) {
    nodes.push_back(&nodes_[tag]);
  }

  auto entity = factory.CreateGeometricEntity(type, nodes);
  entities_.push_back(std::move(entity));
}

void Geometry::RegisterGeometricEntityGroup(const std::string& name,
                                            const std::vector<size_t>& entity_tags) {
  if (entity_groups_.contains(name)) {
    throw std::runtime_error("Entity group already registered");
  };

  std::vector<GeometricEntity*> entities;
  entities.reserve(entity_tags.size());
  for (auto tag : entity_tags) {
    entities.push_back(entities_[tag].get());
  }

  entity_groups_.insert({name, entities});
}

}  // namespace ffea
