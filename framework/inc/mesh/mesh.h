#ifndef FFEA_FRAMEWORK_INC_MESH_MESH_H_
#define FFEA_FRAMEWORK_INC_MESH_MESH_H_

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../geometry/geometry.h"
#include "../geometry/node.h"
#include "./degree_of_freedom.h"
#include "./element.h"
#include "./element_factory.h"

namespace ffea {

class Mesh {
 public:
  Mesh(Geometry& geometry, size_t dofs_per_node);

  size_t number_of_nodes() const;
  size_t number_of_nodes(const std::string& group_name) const;
  size_t number_of_dofs() const;
  size_t number_of_dofs(const std::string& group_name) const;
  size_t dofs_per_node() const;
  void AddElement(const std::string& group_name,
                  GeometricEntity& geometric_entity,
                  const ElementFactory& factory);
  std::vector<Element>& GetElementGroup(const std::string& group_name);
  const std::vector<Element>& GetElementGroup(
      const std::string& group_name) const;
  void SetSolutionOnDofs(const Eigen::VectorXd& solution);
  std::vector<double> GetNodalValues(const std::string& group_name) const;
  std::vector<Coordinates> GetNodalCoords(const std::string& group_name) const;
  const std::vector<Node>& nodes() const;

 private:
  void AddDofs();
  size_t GetDofId(size_t node_id, size_t component_idx) const;
  const std::vector<DegreeOfFreedom>& dofs() const;
  const std::unordered_set<const DegreeOfFreedom*> GetElementGroupDofs(
      const std::string& group_name) const;

  Geometry& geometry_;
  size_t dofs_per_node_;
  std::vector<DegreeOfFreedom> dofs_;
  std::unordered_map<std::string, std::vector<Element>> element_groups_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESH_H_