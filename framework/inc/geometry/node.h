#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_

#include <eigen3/Eigen/Dense>
#include <vector>

#include "./coordinates.h"

namespace ffea {

class Node {
 public:
  Node(size_t id, const Coordinates &coords);

  size_t id() const;
  Coordinates &coords();
  const Coordinates &coords() const;

 private:
  size_t id_;
  Coordinates coords_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_
