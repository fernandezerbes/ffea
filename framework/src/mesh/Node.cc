#include "../../inc/mesh/Node.h"

namespace ffea
{

Node::Node(const Coordinates &coordinates, short number_of_dofs)
  : coordinates_(coordinates),
    dofs_(number_of_dofs)
  {}

Node::~Node() {}

std::vector<DegreeOfFreedom> &Node::dofs()
{
  return dofs_;
}

void Node::set_number_of_dofs(short number_of_dofs) {
  dofs_.resize(number_of_dofs);
}

short Node::number_of_dofs() const {
  return dofs_.size();
}
  
Coordinates &Node::coordinates() {
  return coordinates_;
}

} // namespace ffea
