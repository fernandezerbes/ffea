#include "../../inc/mesh/node.h"

namespace ffea {

Node::Node(size_t id, const Coordinates &coordinates)
    : id_(id), coordinates_(coordinates) {}

Node::~Node() {}

size_t Node::id() const { return id_; }

Coordinates &Node::coordinates() { return coordinates_; }

const Coordinates &Node::coordinates() const { return coordinates_; }

}  // namespace ffea
