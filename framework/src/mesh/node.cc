#include "../../inc/mesh/node.h"

namespace ffea {

Node::Node(size_t id, const Coordinates &coordinates,
           short number_of_dofs)
    : id_(id),
      coordinates_(coordinates),
      dofs_(number_of_dofs) {}

Node::~Node() {}

size_t Node::id() const { return id_; }

Coordinates &Node::coordinates() { return coordinates_; }

std::vector<DegreeOfFreedom> &Node::dofs() { return dofs_; }

size_t Node::number_of_dofs() const { return dofs_.size(); }

}  // namespace ffea
