#include "../../inc/mesh/geometric_entity.h"

namespace ffea {

GeometricEntity::GeometricEntity(
    size_t dimension, const std::vector<Node *> &nodes,
    std::unique_ptr<ShapeFunctions> shape_functions)
    : dimension_(dimension),
      nodes_(nodes),
      shape_functions_(std::move(shape_functions)) {}

GeometricEntity::~GeometricEntity() {}

size_t GeometricEntity::dimension() const { return dimension_; }

size_t GeometricEntity::GetNumberOfNodes() const { return nodes_.size(); }

std::vector<size_t> GeometricEntity::GetNodesIds() const {
  std::vector<size_t> nodes_ids;
  nodes_ids.reserve(GetNumberOfNodes());
  for (const auto &node : nodes_) {
    nodes_ids.push_back(node->id());
  }
  return nodes_ids;
}

Eigen::MatrixXd GeometricEntity::EvaluateShapeFunctions(
    const Coordinates &local_coordinates,
    DerivativeOrder derivative_order) const {
  return shape_functions_->Evaluate(local_coordinates.get(), derivative_order);
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

const std::vector<Node *> &GeometricEntity::nodes() const { return nodes_; }

TwoNodeLine2D::TwoNodeLine2D(const std::vector<Node *> &nodes)
    : GeometricEntity(2, nodes, std::make_unique<Linear1DShapeFunctions>()) {}

TwoNodeLine2D::~TwoNodeLine2D() {}

Eigen::VectorXd TwoNodeLine2D::EvaluateNormal(
    const Coordinates &local_coordinates) const {
  Eigen::MatrixXd jacobian =
      GeometricEntity::EvaluateJacobian(local_coordinates);
  // [1x2] * [2x2] = [1x2] = [dx/dxi, dy/dxi]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(2);
  normal(0) = jacobian(0, 1);
  normal(1) = -jacobian(0, 0);
  return normal;
}

TwoNodeLine3D::TwoNodeLine3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes, std::make_unique<Linear1DShapeFunctions>()) {}

TwoNodeLine3D::~TwoNodeLine3D() {}

Eigen::VectorXd TwoNodeLine3D::EvaluateNormal(
    const Coordinates &local_coordinates) const {}

FourNodeQuad2D::FourNodeQuad2D(const std::vector<Node *> &nodes)
    : GeometricEntity(2, nodes, std::make_unique<Linear2DShapeFunctions>()) {}

FourNodeQuad2D::~FourNodeQuad2D() {}

Eigen::VectorXd FourNodeQuad2D::EvaluateNormal(
    const Coordinates &local_coordinates) const {}

FourNodeQuad3D::FourNodeQuad3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes, std::make_unique<Linear2DShapeFunctions>()) {}

FourNodeQuad3D::~FourNodeQuad3D() {}

Eigen::VectorXd FourNodeQuad3D::EvaluateNormal(
    const Coordinates &local_coordinates) const {
  Eigen::MatrixXd jacobian =
      GeometricEntity::EvaluateJacobian(local_coordinates);
  // [2x4] * [4x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);
  normal(2) = jacobian(0, 0) * jacobian(1, 2) - jacobian(0, 1) * jacobian(1, 0);
  return normal;
}

}  // namespace ffea
