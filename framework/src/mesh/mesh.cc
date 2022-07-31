#include "../../inc/mesh/mesh.h"

namespace ffea {

Mesh::Mesh(short int number_of_dofs_per_node, const std::vector<Node>& nodes)
  : number_of_dofs_per_node_(number_of_dofs_per_node),
    nodes_(nodes)
{}

Mesh::~Mesh() {}

// void Mesh::RegisterElementGroup(
//     std::string group_name, const std::vector<Element>& elements) {
//   element_groups_.insert({group_name, elements});
// }

// const std::vector<Element>& Mesh::GetElementGroup(
//     std::string group_name) const {
//   return element_groups_.at(group_name);
// }

long unsigned int Mesh::number_of_dofs() const {
  return nodes_.size() * number_of_dofs_per_node_;
}

} // namespace ffea

