#include "../../inc/geometry/geometric_entity.h"

#include <stdexcept>
#include <string>

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
  return shape_functions_->Evaluate(local_coordinates, derivative_order);
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
    const Coordinates &local_coordinates,
    const Eigen::MatrixXd &shape_functions_derivatives) const {
  const auto &nodes_coordinates_values = GetNodesCoordinatesValues();
  return shape_functions_derivatives * nodes_coordinates_values;
}

Eigen::MatrixXd GeometricEntity::EvaluateJacobian(
    const Coordinates &local_coordinates) const {
  const auto &shape_functions_derivatives =
      EvaluateShapeFunctions(local_coordinates, DerivativeOrder::kFirst);
  return EvaluateJacobian(local_coordinates, shape_functions_derivatives);
}

Coordinates GeometricEntity::MapLocalToGlobal(
    const Coordinates &local_coordinates) const {
  const auto &shape_functions =
      EvaluateShapeFunctions(local_coordinates, DerivativeOrder::kZeroth);
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
    const Eigen::MatrixXd &jacobian) const {
  std::string type_name = typeid(*this).name();
  throw std::logic_error("Normal is undefined for geometric entity of type " +
                         type_name);
}

Coordinates &GeometricEntity::GetCoordinatesOfNode(size_t node_index) const {
  return nodes_[node_index]->coordinates();
}

const std::vector<Node *> &GeometricEntity::nodes() const { return nodes_; }

TwoNodeLine2D::TwoNodeLine2D(const std::vector<Node *> &nodes)
    : GeometricEntity(2, nodes, std::make_unique<TwoNodeLineShapeFunctions>()) {
}

TwoNodeLine2D::~TwoNodeLine2D() {}

Eigen::VectorXd TwoNodeLine2D::EvaluateNormalVector(
    const Eigen::MatrixXd &jacobian) const {
  // [1x2] * [2x2] = [1x2] = [dx/dxi, dy/dxi]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(2);
  normal(0) = jacobian(0, 1);
  normal(1) = -jacobian(0, 0);
  return normal;
}

double TwoNodeLine2D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  return jacobian.determinant();
}

GeometricEntityType TwoNodeLine2D::type() const {
  return GeometricEntityType::kTwoNodeLine;
}

TwoNodeLine3D::TwoNodeLine3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes, std::make_unique<TwoNodeLineShapeFunctions>()) {
}

TwoNodeLine3D::~TwoNodeLine3D() {}

Eigen::VectorXd TwoNodeLine3D::EvaluateNormalVector(
    const Eigen::MatrixXd &jacobian) const {
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
  double direction_vector_norm = jacobian(0, 0) * jacobian(0, 0) +
                                 jacobian(0, 1) * jacobian(0, 1) +
                                 jacobian(0, 2) * jacobian(0, 2);

  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = 1.0;
  normal(1) = 0.0;
  normal(2) = -jacobian(0, 0) / jacobian(0, 1);

  return normal * std::sqrt(direction_vector_norm / normal.squaredNorm());
}

double TwoNodeLine3D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  double result = std::sqrt(jacobian(0, 0) * jacobian(0, 0) +
                            jacobian(0, 1) * jacobian(0, 1) +
                            jacobian(0, 2) * jacobian(0, 2));
  return result;
}

GeometricEntityType TwoNodeLine3D::type() const {
  return GeometricEntityType::kTwoNodeLine;
}

FourNodeQuad2D::FourNodeQuad2D(const std::vector<Node *> &nodes)
    : GeometricEntity(2, nodes,
                      std::make_unique<FourNodeQuadShapeFunctions>()) {}

FourNodeQuad2D::~FourNodeQuad2D() {}

double FourNodeQuad2D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  return jacobian.determinant();
}

GeometricEntityType FourNodeQuad2D::type() const {
  return GeometricEntityType::kFourNodeQuad;
}

FourNodeQuad3D::FourNodeQuad3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes,
                      std::make_unique<FourNodeQuadShapeFunctions>()) {}

FourNodeQuad3D::~FourNodeQuad3D() {}

Eigen::VectorXd FourNodeQuad3D::EvaluateNormalVector(
    const Eigen::MatrixXd &jacobian) const {
  // [2x4] * [4x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);
  normal(2) = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);
  return normal;
}

double FourNodeQuad3D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  Eigen::VectorXd normal = EvaluateNormalVector(jacobian);
  return normal.norm();
}

GeometricEntityType FourNodeQuad3D::type() const {
  return GeometricEntityType::kFourNodeQuad;
}

EightNodeHex3D::EightNodeHex3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes,
                      std::make_unique<EightNodeHexShapeFunctions>()) {}

EightNodeHex3D::~EightNodeHex3D() {}

double EightNodeHex3D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  return jacobian.determinant();
}

GeometricEntityType EightNodeHex3D::type() const {
  return GeometricEntityType::kEightNodeHex;
}

ThreeNodeTria2D::ThreeNodeTria2D(const std::vector<Node *> &nodes)
    : GeometricEntity(2, nodes,
                      std::make_unique<ThreeNodeTriaShapeFunctions>()) {}

ThreeNodeTria2D::~ThreeNodeTria2D() {}

double ThreeNodeTria2D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  return jacobian.determinant();
}

GeometricEntityType ThreeNodeTria2D::type() const {
  return GeometricEntityType::kThreeNodeTria;
}

ThreeNodeTria3D::ThreeNodeTria3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes,
                      std::make_unique<ThreeNodeTriaShapeFunctions>()) {}

ThreeNodeTria3D::~ThreeNodeTria3D() {}

Eigen::VectorXd ThreeNodeTria3D::EvaluateNormalVector(
    const Eigen::MatrixXd &jacobian) const {
  // [2x3] * [3x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);
  normal(2) = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);
  return normal;
}

double ThreeNodeTria3D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  Eigen::VectorXd normal = EvaluateNormalVector(jacobian);
  return normal.norm();
}

GeometricEntityType ThreeNodeTria3D::type() const {
  return GeometricEntityType::kThreeNodeTria;
}

FourNodeTetra3D::FourNodeTetra3D(const std::vector<Node *> &nodes)
    : GeometricEntity(3, nodes,
                      std::make_unique<FourNodeTetraShapeFunctions>()) {}

FourNodeTetra3D::~FourNodeTetra3D() {}

double FourNodeTetra3D::EvaluateDifferential(
    const Eigen::MatrixXd &jacobian) const {
  return jacobian.determinant();
}

GeometricEntityType FourNodeTetra3D::type() const {
  return GeometricEntityType::kFourNodeTetra;
}

}  // namespace ffea
