#ifndef FFEA_FRAMEWORK_INC_MESH_MESH_H_
#define FFEA_FRAMEWORK_INC_MESH_MESH_H_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "./coordinates.h"
#include "./degree_of_freedom.h"
#include "./element.h"
#include "./element_factory.h"
#include "./geometric_entity.h"
#include "./geometry.h"
#include "./node.h"

namespace ffea {

class Mesh {
 public:
  Mesh(Geometry& geometry, size_t dofs_per_node);
  ~Mesh();

  size_t number_of_dofs() const;
  size_t number_of_nodes() const;
  void AddElement(const std::string& group_name,
                  GeometricEntity& geometric_entity,
                  const ElementFactory& factory);
  std::vector<Element>& GetElementGroup(const std::string& group_name);
  const std::vector<Element>& GetElementGroup(
      const std::string& group_name) const;
  void SetSolutionOnDofs(const Eigen::VectorXd& solution);
  double GetSolutionAtDof(size_t dof_id) const;
  const std::vector<Node> &nodes() const;
  const std::vector<DegreeOfFreedom> &dofs() const;
  size_t dofs_per_node() const;

 private:
  Geometry& geometry_;
  size_t dofs_per_node_;
  std::vector<DegreeOfFreedom> dofs_;
  std::unordered_map<std::string, std::vector<Element>> element_groups_;
  size_t GetDofId(size_t node_id, size_t component_index) const;
  void AddDofs();
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESH_H_