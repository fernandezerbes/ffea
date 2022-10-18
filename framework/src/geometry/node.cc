#include "../../inc/geometry/node.h"

namespace ffea {

Node::Node(size_t tag, const Coordinates &coords)
    : tag_(tag), coords_(coords) {}

size_t Node::tag() const { return tag_; }

Coordinates &Node::coords() { return coords_; }

const Coordinates &Node::coords() const { return coords_; }

}  // namespace ffea
