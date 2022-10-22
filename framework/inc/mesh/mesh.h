#ifndef FFEA_FRAMEWORK_INC_MESH_MESH_H_
#define FFEA_FRAMEWORK_INC_MESH_MESH_H_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../geometry/geometry.h"
#include "../geometry/node.h"
#include "../alias.h"
#include "./degree_of_freedom.h"
#include "./element.h"
#include "./element_factory.h"

namespace ffea {

class Mesh {
 public:
  Mesh(Geometry& geometry, size_t dofs_per_node);

  size_t number_of_nodes() const;
  size_t number_of_nodes(const std::string& group_name) const;
  const std::vector<Node>& nodes() const;
  std::vector<double> nodal_values(const std::string& group_name) const;
  size_t number_of_dofs() const;
  size_t number_of_dofs(const std::string& group_name) const;
  size_t dofs_per_node() const;
  std::vector<Element>& element_group(const std::string& group_name);
  const std::vector<Element>& element_group(
      const std::string& group_name) const;

  void AddElement(const std::string& group_name, GeometricEntity& entity,
                  const ElementFactory& factory);
  void SetSolutionOnDofs(const Vector<double>& solution);

 private:
  size_t dof_tag(size_t node_tag, size_t component_idx) const;
  const std::unordered_set<const DegreeOfFreedom*> element_group_dofs(
      const std::string& group_name) const;

  void AddDofs();

  Geometry& geometry_;
  size_t dofs_per_node_;
  std::vector<DegreeOfFreedom> dofs_;
  std::unordered_map<std::string, std::vector<Element>> element_groups_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESH_H_
