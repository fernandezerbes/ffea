#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "../processor/operator.h"
#include "./coordinates.h"
#include "./degree_of_freedom.h"
#include "./geometric_entity.h"
#include "./integration_point.h"
#include "./node.h"

namespace ffea {

using ConditionFunction =
    std::function<std::vector<double>(const Coordinates &)>;

class Element {
 public:
  Element(GeometricEntity &geometric_entity,
          const std::vector<DegreeOfFreedom *> &dofs,
          const IntegrationPointsGroup &integration_points);
  virtual ~Element();

  std::vector<size_t> GetLocalToGlobalDofIndicesMap() const;
  size_t GetNumberOfDofs() const;
  size_t GetNumberOfNodes() const;
  size_t GetNumberOfDofsPerNode() const;
  Coordinates &GetCoordinatesOfNode(size_t node_index) const;
  Eigen::MatrixXd ComputeStiffness(
      const Eigen::MatrixXd &constitutive_model,
      const DifferentialOperator &differential_operator) const;
  Eigen::VectorXd ComputeRhs(ConditionFunction load) const;
  void SetSolutionOnDofs(const Eigen::VectorXd &solution);
  Eigen::VectorXd GetSolutionFromDofs(size_t component_index) const;
  const std::vector<Node *> &nodes() const;

 private:
  GeometricEntity &geometric_entity_;
  std::vector<DegreeOfFreedom *> dofs_;
  const IntegrationPointsGroup &integration_points_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
