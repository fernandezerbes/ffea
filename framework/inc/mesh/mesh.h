#ifndef FFEA_FRAMEWORK_INC_MESH_MESH_H_
#define FFEA_FRAMEWORK_INC_MESH_MESH_H_

#include <string>
#include <vector>
#include <unordered_map>

#include "./element.h"
#include "./node.h"

namespace ffea {

class Mesh
{
 public:
  Mesh(short int number_of_dofs_per_node, const std::vector<Node>& nodes);
  ~Mesh();

  // void RegisterElementGroup(
  //   std::string group_name, const std::vector<Element>& elements);
  // const std::vector<Element>& GetElementGroup(std::string group_name) const;
  long unsigned int number_of_dofs() const;

  std::vector<Element> dirichlet_boundary_;
  std::vector<Element> neumann_boundary_;
  std::vector<Element> body_;

 private:
  short int number_of_dofs_per_node_;
  std::vector<Node> nodes_;
  // std::unordered_map<std::string, std::vector<Element>> element_groups_;
};

} // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESH_H_