#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "../math/shape_functions.h"
#include "../processor/operator.h"
#include "./coordinates.h"
#include "./degree_of_freedom.h"
#include "./integration_point.h"
#include "./node.h"

namespace ffea {

using ConditionFunction =
    std::function<std::vector<double>(const Coordinates &)>;

class Element {
 public:
  Element(size_t dimension, const std::vector<Node *> &nodes,
          std::shared_ptr<ShapeFunctions> shape_functions,
          IntegrationPointsGroupPtr integration_points);
  virtual ~Element();

  size_t dimension() const;
  std::vector<Node *> &nodes();
  std::vector<int> GetLocalToGlobalDofIndicesMap() const;
  Eigen::MatrixXd ComputeStiffness(
      const Eigen::MatrixXd &constitutive_model,
      const DifferentialOperator &differential_operator) const;
  Eigen::VectorXd ComputeRhs(ConditionFunction load) const;
  size_t GetNumberOfDofsPerNode() const;
  Coordinates &GetCoordinatesOfNode(size_t node_index) const;

 private:
  size_t dimension_;
  std::vector<Node *> nodes_;
  std::shared_ptr<ShapeFunctions> shape_functions_;
  IntegrationPointsGroupPtr integration_points_;

  IntegrationPointsGroupPtr integration_points() const;
  size_t GetNumberOfNodes() const;
  size_t GetNumberOfDofs() const;
  Eigen::MatrixXd GetNodesCoordinatesValues() const;
  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coordinates,
      DerivativeOrder derivative_order = DerivativeOrder::kZeroth) const;
  Eigen::MatrixXd EvaluateJacobian(const Coordinates &local_coordinates) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coordinates) const;
};

class ElementFactory {
 public:
  ElementFactory(size_t dimension,
                 std::shared_ptr<ShapeFunctions> shape_functions,
                 std::shared_ptr<QuadratureRule> integration_rule);

  Element CreateElement(const std::vector<Node *> &nodes) const;

 private:
  size_t dimension_;
  std::shared_ptr<ShapeFunctions> shape_functions_;
  IntegrationPointsGroupPtr integration_points_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
