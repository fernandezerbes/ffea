#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_

#include <eigen3/Eigen/Dense>
#include <vector>

#include "./coordinates.h"

namespace ffea {

class Node {
 public:
  Node(size_t id, const Coordinates &coordinates);
  ~Node();

  size_t id() const;
  Coordinates &coordinates();
  const Coordinates &coordinates() const;

 private:
  size_t id_;
  Coordinates coordinates_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_NODE_H_
