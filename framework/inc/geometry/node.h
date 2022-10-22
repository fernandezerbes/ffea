#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_

#include "./coordinates.h"

namespace ffea {

class Node {
 public:
  Node(size_t tag, const Coordinates &coords);

  size_t tag() const;
  Coordinates &coords();
  const Coordinates &coords() const;

 private:
  size_t tag_;
  Coordinates coords_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_
