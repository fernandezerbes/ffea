#include "../../inc/mesh/geometric_entity.h"

namespace ffea {

GeometricEntity::GeometricEntity(size_t dimension,
                                 const std::vector<Node *> &nodes,
                                 const ShapeFunctions &shape_functions)
    : dimension_(dimension), nodes_(nodes), shape_functions_(shape_functions) {}

GeometricEntity::~GeometricEntity() {}

size_t GeometricEntity::dimension() const { return dimension_; }

size_t GeometricEntity::GetNumberOfNodes() const { return nodes_.size(); }

Eigen::MatrixXd GeometricEntity::EvaluateShapeFunctions(
    const Coordinates &local_coordinates,
    DerivativeOrder derivative_order) const {
  return shape_functions_.Evaluate(local_coordinates.get(), derivative_order);
}

Eigen::MatrixXd GeometricEntity::GetNodesCoordinatesValues() const {
  Eigen::MatrixXd coordinates_values(GetNumberOfNodes(), dimension_);
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    const auto &coordinates = node->coordinates();
    for (size_t dimension_index = 0; dimension_index < dimension_;
         dimension_index++) {
      coordinates_values(node_index, dimension_index) =
          coordinates.get(dimension_index);
    }
  }
  return coordinates_values;
}

Eigen::MatrixXd GeometricEntity::EvaluateJacobian(
    const Coordinates &local_coordinates) const {
  const auto &shape_functions_derivatives =
      EvaluateShapeFunctions(local_coordinates, DerivativeOrder::kFirst);
  const auto &nodes_coordinates_values = GetNodesCoordinatesValues();
  return shape_functions_derivatives * nodes_coordinates_values;
}

Eigen::MatrixXd GeometricEntity::EvaluateDifferentialDomain(
    const Coordinates &local_coordinates) const {
  // const auto &shape_functions_derivatives =
  //     EvaluateShapeFunctions(local_coordinates, DerivativeOrder::kFirst);
  // const auto &nodes_coordinates_values = GetNodesCoordinatesValues();
  // return shape_functions_derivatives * nodes_coordinates_values;
}

Coordinates GeometricEntity::MapLocalToGlobal(
    const Coordinates &local_coordinates,
    const Eigen::MatrixXd &shape_functions) const {
  // TODO Maybe this should be always 3: std::array<double, 3>
  std::vector<double> xyz(dimension_, 0.0);
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    const auto &coordinates = node->coordinates();
    // TODO Dimension index will be always less than 3
    // Maybe we can always use the three dimensions
    for (size_t dimension_index = 0; dimension_index < dimension_;
         dimension_index++) {
      xyz[dimension_index] +=
          shape_functions(0, node_index) * coordinates.get(dimension_index);
    }
  }
  // TODO this can be avoided if we create the coordinates before
  return Coordinates(xyz);
}

Coordinates &GeometricEntity::GetCoordinatesOfNode(size_t node_index) const {
  return nodes_[node_index]->coordinates();
}



}  // namespace ffea
