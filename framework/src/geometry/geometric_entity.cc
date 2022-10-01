#include "../../inc/geometry/geometric_entity.h"

#include <stdexcept>
#include <string>

namespace ffea {

GeometricEntity::GeometricEntity(
    GeometricEntityType type, size_t dimensions,
    const std::vector<Node *> &nodes,
    std::unique_ptr<ShapeFunctions> shape_functions)
    : type_(type),
      dimension_(dimensions),
      nodes_(nodes),
      shape_functions_(std::move(shape_functions)) {}

GeometricEntityType GeometricEntity::type() const { return type_; }

size_t GeometricEntity::dimensions() const { return dimension_; }

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
    const Coordinates &local_coords,
    DerivativeOrder derivative_order) const {
  return shape_functions_->Evaluate(local_coords, derivative_order);
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
    const Coordinates &local_coords,
    const Eigen::MatrixXd &shape_functions_derivatives) const {
  const auto &nodes_coordinates_values = GetNodesCoordinatesValues();
  return shape_functions_derivatives * nodes_coordinates_values;
}

Eigen::MatrixXd GeometricEntity::EvaluateJacobian(
    const Coordinates &local_coords) const {
  const auto &shape_functions_derivatives =
      EvaluateShapeFunctions(local_coords, DerivativeOrder::kFirst);
  return EvaluateJacobian(local_coords, shape_functions_derivatives);
}

Coordinates GeometricEntity::MapLocalToGlobal(
    const Coordinates &local_coords) const {
  const auto &shape_functions =
      EvaluateShapeFunctions(local_coords, DerivativeOrder::kZeroth);
  return MapLocalToGlobal(shape_functions);
}

Coordinates GeometricEntity::MapLocalToGlobal(
    const Eigen::MatrixXd &shape_functions_at_point) const {
  std::array<double, 3> xyz{};
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    const auto &coordinates = node->coordinates();
    for (size_t dimension_index = 0; dimension_index < dimension_;
         dimension_index++) {
      xyz[dimension_index] += shape_functions_at_point(0, node_index) *
                              coordinates.get(dimension_index);
    }
  }
  return Coordinates(xyz);
}

Eigen::VectorXd GeometricEntity::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  std::string type_name = typeid(*this).name();
  throw std::logic_error("Normal is undefined for geometric entity of type " +
                         type_name);
}

Coordinates &GeometricEntity::GetCoordinatesOfNode(size_t node_index) const {
  return nodes_[node_index]->coordinates();
}

const std::vector<Node *> &GeometricEntity::nodes() const { return nodes_; }

Line2D::Line2D(GeometricEntityType type, const std::vector<Node *> &nodes,
               std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 2, nodes, std::move(shape_functions)) {}

Eigen::VectorXd Line2D::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  // [1x2] * [2x2] = [1x2] = [dx/dxi, dy/dxi]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(2);
  normal(0) = jacobian(0, 1);
  normal(1) = -jacobian(0, 0);
  return normal;
}

double Line2D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant();
}

Line3D::Line3D(GeometricEntityType type, const std::vector<Node *> &nodes,
               std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 3, nodes, std::move(shape_functions)) {}

Eigen::VectorXd Line3D::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  /*
  - Jacobian has the form: [1x2] * [2x3] = [1x3] = [dx/dxi, dy/dxi, dz/dxi]
  - Line vector is u = [dx/dxi, dy/dxi, dz/dxi] = [a, b, c], which is
    perpendicular to a plane a * x + b * y + c * z = 0
  - Replacing th dx/dxi * x + dy/dxi * y + dz/dxi * z = 0 and choosing any x and
    y, e.g. x = 1 and y = 0 we find z as
    z = - dx/dxi / dz/dxi
  - Normal vector to lying on plane is n = [x, y, z] = [1, 0, -dx/dxi / dz/dxi]
  - Scale the vector to have the same length as u
  */
  const auto &jacobian = EvaluateJacobian(local_coords);
  double direction_vector_norm = jacobian(0, 0) * jacobian(0, 0) +
                                 jacobian(0, 1) * jacobian(0, 1) +
                                 jacobian(0, 2) * jacobian(0, 2);

  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = 1.0;
  normal(1) = 0.0;
  normal(2) = -jacobian(0, 0) / jacobian(0, 1);

  return normal * std::sqrt(direction_vector_norm / normal.squaredNorm());
}

double Line3D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  double result = std::sqrt(jacobian(0, 0) * jacobian(0, 0) +
                            jacobian(0, 1) * jacobian(0, 1) +
                            jacobian(0, 2) * jacobian(0, 2));
  return result;
}

Quad2D::Quad2D(GeometricEntityType type, const std::vector<Node *> &nodes,
               std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 2, nodes, std::move(shape_functions)) {}

double Quad2D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant();
}

Quad3D::Quad3D(GeometricEntityType type, const std::vector<Node *> &nodes,
               std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 3, nodes, std::move(shape_functions)) {}

Eigen::VectorXd Quad3D::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  // [2x4] * [4x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  const auto &jacobian = EvaluateJacobian(local_coords);
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);
  normal(2) = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);
  return normal;
}

double Quad3D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  Eigen::VectorXd normal = EvaluateNormalVector(local_coords);
  return normal.norm();
}

Hex3D::Hex3D(GeometricEntityType type, const std::vector<Node *> &nodes,
             std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 3, nodes, std::move(shape_functions)) {}

double Hex3D::EvaluateDifferential(const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant();
}

Tria2D::Tria2D(GeometricEntityType type, const std::vector<Node *> &nodes,
               std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 2, nodes, std::move(shape_functions)) {}

double Tria2D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  const auto& jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant() / 2.0;
}

Tria3D::Tria3D(GeometricEntityType type, const std::vector<Node *> &nodes,
               std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 3, nodes, std::move(shape_functions)) {}

Eigen::VectorXd Tria3D::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  // [2x3] * [3x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  const auto& jacobian = EvaluateJacobian(local_coords);
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);
  normal(2) = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);
  return normal;
}

double Tria3D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  Eigen::VectorXd normal = EvaluateNormalVector(local_coords);
  return normal.norm() / 2.0;
}

Tetra3D::Tetra3D(GeometricEntityType type, const std::vector<Node *> &nodes,
                 std::unique_ptr<ShapeFunctions> shape_functions)
    : GeometricEntity(type, 3, nodes, std::move(shape_functions)) {}

double Tetra3D::EvaluateDifferential(
    const Coordinates &local_coords) const {
  const auto& jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant() / 6.0;
}

}  // namespace ffea
