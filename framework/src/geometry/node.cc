#include "../../inc/geometry/node.h"

namespace ffea {

Node::Node(size_t id, const Coordinates &coords) : id_(id), coords_(coords) {}

size_t Node::id() const { return id_; }

Coordinates &Node::coords() { return coords_; }

const Coordinates &Node::coords() const { return coords_; }

}  // namespace ffea
