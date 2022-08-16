#include "../../inc/mesh/node.h"

namespace ffea {

Node::Node(size_t id, const Coordinates &coordinates, short number_of_dofs)
    : id_(id), coordinates_(coordinates), dofs_() {
  for (size_t component_index = 0; component_index < number_of_dofs;
       component_index++) {
    size_t dof_index = id * number_of_dofs + component_index;
    dofs_.emplace_back(dof_index);
  }
}

Node::~Node() {}

size_t Node::id() const { return id_; }

Coordinates &Node::coordinates() { return coordinates_; }

std::vector<DegreeOfFreedom> &Node::dofs() { return dofs_; }

size_t Node::number_of_dofs() const { return dofs_.size(); }

void Node::SetSolutionOnDofs(const Eigen::VectorXd &solution) {
  for (auto &dof : dofs_) {
    auto value = solution(dof.local_id());
    dof.set_value(value);
  }
}

double Node::GetSolutionOfDof(size_t local_dof_index) const {
  return dofs_[local_dof_index].value();
}

}  // namespace ffea
