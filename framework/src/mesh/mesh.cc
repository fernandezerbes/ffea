#include "../../inc/mesh/mesh.h"

namespace ffea {

Mesh::Mesh(size_t number_of_dofs_per_node)
    : number_of_dofs_per_node_(number_of_dofs_per_node),
      nodes_(),
      element_groups_(),
      cached_element_factories_() {}

Mesh::~Mesh() {}

std::vector<Element>& Mesh::GetElementGroup(const std::string& group_name) {
  return element_groups_.at(group_name);
}

std::vector<Node>& Mesh::nodes() {
  return nodes_;
}  // TODO See how not to break encapsulation

const ElementFactory& Mesh::GetElementFactory(ElementType element_type) {
  // TODO Simplify with cached_element_factories_.try_emplace
  if (!cached_element_factories_.contains(element_type)) {
    cached_element_factories_.insert({element_type, element_type});
  }

  return cached_element_factories_.at(element_type);
}

void Mesh::AddNode(const std::array<double, 3>& xyz) {
  size_t id = number_of_nodes();
  // TODO Improve this std::vector<double>(xyz.begin(), xyz.end())
  nodes_.emplace_back(id, std::vector<double>(xyz.begin(), xyz.end()),
                      number_of_dofs_per_node_);
}

void Mesh::AddElement(ElementType element_type, const std::string& group_name,
                      const std::vector<size_t>& node_ids) {
  const auto& element_factory = GetElementFactory(element_type);
  std::vector<Node*> element_nodes;

  // TODO See if this can be improved with a span, view or something like that
  element_nodes.reserve(node_ids.size());
  for (size_t id : node_ids) {
    element_nodes.push_back(&nodes_[id]);
  }
  auto element = element_factory.CreateElement(element_nodes);

  if (element_groups_.contains(group_name)) {
    element_groups_.at(group_name).push_back(element);
  } else {
    std::vector<Element> elements{element};
    element_groups_.insert({group_name, elements});
  }
}

size_t Mesh::number_of_dofs() const {
  return number_of_nodes() * number_of_dofs_per_node_;
}

size_t Mesh::number_of_nodes() const { return nodes_.size(); }

}  // namespace ffea
