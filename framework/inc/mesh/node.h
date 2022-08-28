#ifndef FFEA_FRAMEWORK_INC_MESH_NODE_H_
#define FFEA_FRAMEWORK_INC_MESH_NODE_H_

#include <vector>

#include <eigen3/Eigen/Dense>

#include "./coordinates.h"
#include "./degree_of_freedom.h"

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

#endif  // FFEA_FRAMEWORK_INC_MESH_NODE_H_
