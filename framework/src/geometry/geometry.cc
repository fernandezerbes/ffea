#include "../../inc/geometry/geometry.h"

#include <stdexcept>
namespace ffea {

Geometry::Geometry()
    : nodes_(), geometric_entities_(), geometric_entities_groups_() {}

Geometry::~Geometry() {}

std::vector<GeometricEntity*>& Geometry::GetGeometricEntityGroup(
    const std::string& group_name) {
  return geometric_entities_groups_.at(group_name);
}

const std::vector<GeometricEntity*>& Geometry::GetGeometricEntityGroup(
    const std::string& group_name) const {
  return geometric_entities_groups_.at(group_name);
}

void Geometry::AddNode(const std::array<double, 3>& xyz) {
  size_t id = number_of_nodes();
  nodes_.emplace_back(id, xyz);
}

void Geometry::AddGeometricEntity(GeometricEntityType type,
                                  const std::vector<size_t>& node_ids,
                                  const GeometricEntityFactory& factory) {
  std::vector<Node*> nodes;
  nodes.reserve(node_ids.size());
  for (size_t id : node_ids) {
    nodes.push_back(&nodes_[id]);
  }

  auto geometric_entity = factory.CreateGeometricEntity(type, nodes);
  geometric_entities_.push_back(std::move(geometric_entity));
}

void Geometry::RegisterGeometricEntityGroup(
    const std::string& group_name,
    const std::vector<size_t>& geometric_entities_ids) {
  if (geometric_entities_groups_.contains(group_name)) {
    throw std::runtime_error("Entity group already registered");
  };

  std::vector<GeometricEntity*> geometric_entities;
  geometric_entities.reserve(geometric_entities_ids.size());
  for (auto id : geometric_entities_ids) {
    geometric_entities.push_back(geometric_entities_[id].get());
  }

  geometric_entities_groups_.insert({group_name, geometric_entities});
}

size_t Geometry::number_of_nodes() const { return nodes_.size(); }

const std::vector<Node>& Geometry::nodes() const { return nodes_; }

}  // namespace ffea
