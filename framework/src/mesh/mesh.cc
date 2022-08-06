#include "../../inc/mesh/mesh.h"

namespace ffea {

Mesh::Mesh(short int number_of_dofs_per_node, const std::vector<Node>& nodes)
    : number_of_dofs_per_node_(number_of_dofs_per_node), nodes_(nodes) {}

Mesh::~Mesh() {}

void Mesh::RegisterElementGroup(ElementGroupType group_type,
                                const std::string& group_name,
                                const std::vector<Element>& elements) {
  switch (group_type) {
    case ElementGroupType::kBodyElements:
      bodies_.insert({group_name, elements});
      break;
    case ElementGroupType::kDirichletBoundaryElements:
      dirichlet_boundaries_.insert({group_name, elements});
      break;
    case ElementGroupType::kNeumannBoundaryElements:
      neumann_boundaries_.insert({group_name, elements});
      break;
    default:
      break;
  }
}

std::vector<Element>& Mesh::GetElementGroup(ElementGroupType group_type,
                                            const std::string& group_name) {
  switch (group_type) {
    case ElementGroupType::kBodyElements:
      return bodies_.at(group_name);
    case ElementGroupType::kDirichletBoundaryElements:
      return dirichlet_boundaries_.at(group_name);
    case ElementGroupType::kNeumannBoundaryElements:
      return neumann_boundaries_.at(group_name);
    default:
      break;
  }
}

long unsigned int Mesh::number_of_dofs() const {
  return nodes_.size() * number_of_dofs_per_node_;
}

}  // namespace ffea
