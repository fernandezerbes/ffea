#ifndef FFEA_FRAMEWORK_INC_GEOMETRICENTITY_H_
#define FFEA_FRAMEWORK_INC_GEOMETRICENTITY_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "../math/shape_functions.h"
#include "./coordinates.h"
#include "./node.h"

namespace ffea {

class GeometricEntity {
 public:
  GeometricEntity(size_t dimension, const std::vector<Node *> &nodes,
                  const ShapeFunctions &shape_functions);
  virtual ~GeometricEntity();

  size_t dimension() const;
  size_t GetNumberOfNodes() const;
  Coordinates &GetCoordinatesOfNode(size_t node_index) const;
  virtual Eigen::MatrixXd ComputeDifferentialDomain() const = 0;

 private:
  size_t dimension_;
  std::vector<Node *> nodes_;
  const ShapeFunctions &shape_functions_;
  Eigen::MatrixXd GetNodesCoordinatesValues() const;
  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coordinates,
      DerivativeOrder derivative_order = DerivativeOrder::kZeroth) const;
  Eigen::MatrixXd EvaluateJacobian(const Coordinates &local_coordinates) const;
  Eigen::MatrixXd EvaluateDifferentialDomain(
      const Coordinates &local_coordinates) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coordinates,
                               const Eigen::MatrixXd &shape_functions) const;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRICENTITY_H_
