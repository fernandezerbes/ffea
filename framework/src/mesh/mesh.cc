#include "../../inc/mesh/mesh.h"

namespace ffea {

Mesh::Mesh(Geometry& geometry, size_t dofs_per_node)
    : geometry_(geometry),
      dofs_per_node_(dofs_per_node),
      dofs_(),
      element_groups_() {
  AddDofs();
}

Mesh::~Mesh() {}

void Mesh::AddDofs() {
  dofs_.reserve(geometry_.number_of_nodes() * dofs_per_node_);
  for (size_t node_id = 0; node_id < geometry_.number_of_nodes(); node_id++) {
    for (size_t component_index = 0; component_index < dofs_per_node_;
         component_index++) {
      auto dof_id = GetDofId(node_id, component_index);
      dofs_.emplace_back(dof_id);
    }
  }
}

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
      auto dof_id = GetDofId(node_id, component_index);
      element_dofs.push_back(&dofs_[dof_id]);
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
  for (auto& dof : dofs_) {
    dof.set_value(solution);
  }
}

size_t Mesh::GetDofId(size_t node_id, size_t component_index) const {
  return node_id * dofs_per_node_ + component_index;
}

const std::vector<Node>& Mesh::nodes() const { return geometry_.nodes(); }

const std::vector<DegreeOfFreedom>& Mesh::dofs() const { return dofs_; }

size_t Mesh::dofs_per_node() const { return dofs_per_node_; }

double Mesh::GetSolutionAtDof(size_t dof_id) const {
  return dofs_[dof_id].value();
}

}  // namespace ffea
