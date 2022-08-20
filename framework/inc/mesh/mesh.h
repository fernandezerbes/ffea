#ifndef FFEA_FRAMEWORK_INC_MESH_MESH_H_
#define FFEA_FRAMEWORK_INC_MESH_MESH_H_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "./coordinates.h"
#include "./element.h"
#include "./element_factory.h"
#include "./node.h"

namespace ffea {

class Mesh {
 public:
  Mesh(size_t number_of_dofs_per_node);
  ~Mesh();

  size_t number_of_dofs() const;
  size_t number_of_nodes() const;
  void AddNode(const std::array<double, 3>& xyz);
  void AddElement(ElementType element_type, const std::string& group_name,
                  const std::vector<size_t>& node_ids);
  std::vector<Element>& GetElementGroup(const std::string& group_name);
  std::vector<Node>& nodes();

 private:
  const ElementFactory& GetElementFactory(ElementType element_type);

  size_t number_of_dofs_per_node_;
  std::vector<Node> nodes_;
  std::unordered_map<std::string, std::vector<Element>> element_groups_;
  std::unordered_map<ElementType, ElementFactory> cached_element_factories_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESH_H_