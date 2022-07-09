#ifndef FFEA_FRAMEWORK_INC_MESH_NODE_H_
#define FFEA_FRAMEWORK_INC_MESH_NODE_H_

#include <vector>

#include "./Coordinates.h"
#include "./DegreeOfFreedom.h"

namespace ffea
{

class Node {
 public:
  Node(const Coordinates& coordinates, short number_of_dofs = 0);
  ~Node();

  std::vector<DegreeOfFreedom> &dofs();
  void set_number_of_dofs(short number_of_dofs);
  short number_of_dofs() const;
  Coordinates &coordinates();

 private:
  Coordinates coordinates_;
  std::vector<DegreeOfFreedom> dofs_;

};

} // namespace ffea

#endif // FFEA_FRAMEWORK_INC_MESH_NODE_H_
