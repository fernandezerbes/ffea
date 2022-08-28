#include "../../inc/mesh/geometry.h"

namespace ffea {

Geometry::Geometry() : nodes_(), geometric_entities_groups_() {}

Geometry::~Geometry() {}

std::vector<std::shared_ptr<GeometricEntity>>&
Geometry::GetGeometricEntityGroup(const std::string& group_name) {
  return geometric_entities_groups_.at(group_name);
}

const std::vector<std::shared_ptr<GeometricEntity>>&
Geometry::GetGeometricEntityGroup(const std::string& group_name) const {
  return geometric_entities_groups_.at(group_name);
}

void Geometry::AddNode(const std::array<double, 3>& xyz) {
  size_t id = number_of_nodes();
  // TODO Improve this std::vector<double>(xyz.begin(), xyz.end())
  nodes_.emplace_back(id, std::vector<double>(xyz.begin(), xyz.end()));
}

void Geometry::AddGeometricEntity(GeometricEntityType type,
                                  const std::string& group_name,
                                  const std::vector<size_t>& node_ids,
                                  const GeometricEntityFactory& factory) {
  std::vector<Node*> nodes;

  // TODO See if this can be improved with a span, view or something like that
  nodes.reserve(node_ids.size());
  for (size_t id : node_ids) {
    nodes.push_back(&nodes_[id]);
  }
  auto element = factory.CreateGeometricEntity(type, nodes);

  if (geometric_entities_groups_.contains(group_name)) {
    geometric_entities_groups_.at(group_name).push_back(element);
  } else {
    std::vector<std::shared_ptr<GeometricEntity>> elements{element};
    geometric_entities_groups_.insert({group_name, elements});
  }
}

size_t Geometry::number_of_nodes() const { return nodes_.size(); }

const std::vector<Node>& Geometry::nodes() const { return nodes_; }

}  // namespace ffea
