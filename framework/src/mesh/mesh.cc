#include "../../inc/mesh/mesh.h"

namespace ffea {

Mesh::Mesh(Geometry& geometry, size_t dofs_per_node)
    : geometry_(geometry),
      dofs_per_node_(dofs_per_node),
      dofs_(),
      element_groups_() {}

Mesh::~Mesh() {}

std::vector<Element>& Mesh::GetElementGroup(const std::string& group_name) {
  return element_groups_.at(group_name);
}

const std::vector<Element>& Mesh::GetElementGroup(
    const std::string& group_name) const {
  return element_groups_.at(group_name);
}

void Mesh::AddElement(const std::string& group_name,
                      GeometricEntity& geometric_entity,
                      const ElementFactory& factory) {
  std::vector<DegreeOfFreedom*> element_dofs;
  element_dofs.reserve(geometric_entity.GetNumberOfNodes() * dofs_per_node_);

  for (size_t node_id : geometric_entity.GetNodesIds()) {
    for (size_t component_index = 0; component_index < dofs_per_node_;
         component_index++) {
      element_dofs.push_back(&dofs_[GetDofId(node_id, component_index)]);
    }
  }

  auto element = factory.CreateElement(geometric_entity, element_dofs);

  if (element_groups_.contains(group_name)) {
    element_groups_.at(group_name).push_back(element);
  } else {
    std::vector<Element> elements{element};
    element_groups_.insert({group_name, elements});
  }
}

size_t Mesh::number_of_dofs() const { return dofs_.size(); }

size_t Mesh::number_of_nodes() const { return geometry_.number_of_nodes(); }

void Mesh::SetSolutionOnDofs(const Eigen::VectorXd& solution) {
  for (auto& element : GetElementGroup("surface")) {
    element.SetSolutionOnDofs(solution);
  }
}

size_t Mesh::GetDofId(size_t node_id, size_t component_index) const {
  return node_id * dofs_per_node_ + component_index;
}

}  // namespace ffea
