#include "../../inc/mesh/mesh.h"

namespace ffea {

Mesh::Mesh(Geometry& geometry, size_t dofs_per_node)
    : geometry_(geometry), dofs_per_node_(dofs_per_node) {
  AddDofs();
}

size_t Mesh::number_of_nodes() const { return geometry_.number_of_nodes(); }

size_t Mesh::number_of_nodes(const std::string& group_name) const {
  std::unordered_set<size_t> group_node_tags;
  for (const auto& element : element_group(group_name)) {
    const auto& element_node_tags = element.node_tags();
    group_node_tags.insert(element_node_tags.begin(), element_node_tags.end());
  }
  return group_node_tags.size();
}

const std::vector<Node>& Mesh::nodes() const { return geometry_.nodes(); }

std::vector<double> Mesh::nodal_values(const std::string& group_name) const {
  const auto& dofs = element_group_dofs(group_name);
  std::vector<double> values(dofs.size());
  for (const auto& dof : dofs) {
    values[dof->tag()] = dof->value();
  }
  return values;
}

size_t Mesh::number_of_dofs() const { return dofs_.size(); }

size_t Mesh::number_of_dofs(const std::string& group_name) const {
  std::unordered_set<size_t> unique_dof_tags;
  for (const auto& element : element_group(group_name)) {
    const auto& dof_tags = element.dof_tags();
    unique_dof_tags.insert(dof_tags.begin(), dof_tags.end());
  }
  return unique_dof_tags.size();
}

size_t Mesh::dofs_per_node() const { return dofs_per_node_; }

std::vector<Element>& Mesh::element_group(const std::string& group_name) {
  return element_groups_.at(group_name);
}

void Mesh::AddElement(const std::string& group_name, GeometricEntity& geometric_entity,
                      const ElementFactory& factory) {
  std::vector<DegreeOfFreedom*> element_dofs;
  element_dofs.reserve(geometric_entity.number_of_nodes() * dofs_per_node_);

  for (const size_t node_idx : geometric_entity.node_tags()) {
    for (size_t component_idx = 0; component_idx < dofs_per_node_; component_idx++) {
      auto& dof = dofs_[dof_tag(node_idx, component_idx)];
      element_dofs.push_back(&dof);
    }
  }

  auto element = factory.CreateElement(geometric_entity, element_dofs);

  if (element_groups_.contains(group_name)) {
    element_groups_.at(group_name).push_back(element);
  } else {
    std::vector<Element> const elements{element};
    element_groups_.insert({group_name, elements});
  }
}

void Mesh::SetSolutionOnDofs(const Vector<double>& solution) {
  for (auto& dof : dofs_) {
    dof.set_value(solution);
  }
}

const std::vector<Element>& Mesh::element_group(const std::string& group_name) const {
  return element_groups_.at(group_name);
}

size_t Mesh::dof_tag(size_t node_idx, size_t component_idx) const {
  return node_idx * dofs_per_node_ + component_idx;
}

std::unordered_set<const DegreeOfFreedom*> Mesh::element_group_dofs(
    const std::string& group_name) const {
  std::unordered_set<const DegreeOfFreedom*> group_dofs;
  for (const auto& element : element_group(group_name)) {
    const auto& element_dofs = element.dofs();
    group_dofs.insert(element_dofs.begin(), element_dofs.end());
  }
  return group_dofs;
}

void Mesh::AddDofs() {
  dofs_.reserve(geometry_.number_of_nodes() * dofs_per_node_);
  for (size_t node_idx = 0; node_idx < geometry_.number_of_nodes(); node_idx++) {
    for (size_t component_idx = 0; component_idx < dofs_per_node_; component_idx++) {
      dofs_.emplace_back(dof_tag(node_idx, component_idx));
    }
  }
}

}  // namespace ffea
