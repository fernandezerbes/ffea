#ifndef FFEA_FRAMEWORK_INC_MESH_NODE_H_
#define FFEA_FRAMEWORK_INC_MESH_NODE_H_

#include <vector>

#include <eigen3/Eigen/Dense>

#include "./coordinates.h"
#include "./degree_of_freedom.h"

namespace ffea {

class Node {
 public:
  Node(size_t id, const Coordinates &coordinates,
       short number_of_dofs = 0);
  ~Node();

  size_t id() const;
  Coordinates &coordinates();
  std::vector<DegreeOfFreedom> &dofs();
  size_t number_of_dofs() const;
  void SetSolutionOnDofs(const Eigen::VectorXd &solution);
  double GetSolutionOfDof(size_t local_dof_index) const;

 private:
  size_t id_;
  Coordinates coordinates_;
  std::vector<DegreeOfFreedom> dofs_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_NODE_H_
