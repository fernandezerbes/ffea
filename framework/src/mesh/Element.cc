#include "../../inc/mesh/Element.h"


namespace ffea
{

Element::Element(const std::vector<Node*> &nodes) : nodes_(nodes) {}

Element::~Element() {}

const std::vector<Node*> &Element::nodes() const {
  return nodes_;
}

std::vector<Coordinates*> Element::node_coordinates() const {
  std::vector<Coordinates*> node_coordinates;
  node_coordinates.reserve(nodes_.size());
  
  for (const auto &node: nodes_) {
    node_coordinates.push_back(&node->coordinates());
  }

  return node_coordinates;
}

std::vector<DegreeOfFreedom*> Element::dofs() const {
  std::vector<DegreeOfFreedom*> dofs;
  dofs.reserve(nodes_.size() * nodes_[0]->number_of_dofs());

  for (const auto &node: nodes_) {
    for (auto &dof: node->dofs()) {
      dofs.push_back(&dof);
    }
  }

  return dofs;
}

size_t Element::number_of_nodes() const {
  return nodes_.size();
}

size_t Element::number_of_dofs() const {
  return number_of_nodes() * nodes_[0]->number_of_dofs();
}

}
