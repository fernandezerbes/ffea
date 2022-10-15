#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../geometry/node.h"
#include "../model/constitutive_model.h"
#include "../model/types.h"
#include "./degree_of_freedom.h"
#include "./integration_point.h"

namespace ffea {

class Element {
 public:
  Element(GeometricEntity &geometric_entity,
          const std::vector<DegreeOfFreedom *> &dofs,
          const IntegrationPointsGroup &integration_points);

  GeometricEntityType GetGeometricEntityType() const;
  std::vector<size_t> GetLocalToGlobalDofIndicesMap() const;
  size_t GetNumberOfDofs() const;
  size_t GetNumberOfNodes() const;
  size_t GetNumberOfDofsPerNode() const;
  Coordinates &GetCoordinatesOfNode(size_t node_idx) const;
  Eigen::MatrixXd EvaluateJacobian(
      const Coordinates &local_coords,
      const Eigen::MatrixXd &shape_functions_derivatives) const;
  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coords,
      DerivativeOrder derivative_order = DerivativeOrder::kZeroth) const;
  // TODO Reuse jacobian where possible in these functions
  Eigen::VectorXd EvaluateNormalVector(const Coordinates &local_coords) const;
  double EvaluateDifferential(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(
      const Eigen::MatrixXd &shape_functions_at_point) const;
  void SetSolutionOnDofs(const Eigen::VectorXd &solution);
  Eigen::VectorXd GetSolutionFromDofs(size_t component_idx) const;
  const std::vector<Node *> &nodes() const;
  const IntegrationPointsGroup &integration_points() const;

 private:
  GeometricEntity &geometric_entity_;
  std::vector<DegreeOfFreedom *> dofs_;
  const IntegrationPointsGroup &integration_points_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
