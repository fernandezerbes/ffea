#ifndef FFEA_FRAMEWORK_INC_MESH_MESH_H_
#define FFEA_FRAMEWORK_INC_MESH_MESH_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "./coordinates.h"
#include "./element.h"
#include "./node.h"

namespace ffea {

enum class ElementGroupType{
  kBodyElements,
  kDirichletBoundaryElements,
  kNeumannBoundaryElements
};

class Mesh {
 public:
  Mesh(short int number_of_dofs_per_node, const std::vector<Node>& nodes);
  ~Mesh();

  void RegisterElementGroup(ElementGroupType group_type,
                            const std::string& group_name,
                            const std::vector<Element> &elements);
  std::vector<Element>& GetElementGroup(
      ElementGroupType group_type,
      const std::string& group_name);
  long unsigned int number_of_dofs() const;

 private:
  short int number_of_dofs_per_node_;
  std::vector<Node> nodes_;
  std::unordered_map<std::string, std::vector<Element>> bodies_;
  std::unordered_map<std::string, std::vector<Element>> dirichlet_boundaries_;
  std::unordered_map<std::string, std::vector<Element>> neumann_boundaries_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESH_H_