#include "../../inc/geometry/geometric_entity.h"

#include <stdexcept>
#include <string>

namespace ffea {

GeometricEntity::GeometricEntity(GeometricEntityType type, size_t dimensions,
                                 const std::vector<Node *> &nodes)
    : type_(type), dimension_(dimensions), nodes_(nodes) {}

GeometricEntityType GeometricEntity::type() const { return type_; }

size_t GeometricEntity::dimensions() const { return dimension_; }

size_t GeometricEntity::number_of_nodes() const { return nodes_.size(); }

std::vector<size_t> GeometricEntity::GetNodeTags() const {
  std::vector<size_t> node_tags;
  node_tags.reserve(number_of_nodes());
  for (const auto &node : nodes_) {
    node_tags.push_back(node->tag());
  }
  return node_tags;
}

Eigen::MatrixXd GeometricEntity::EvaluateShapeFunctions(
    const Coordinates &local_coords, DerivativeOrder derivative_order) const {
  switch (derivative_order) {
    case DerivativeOrder::kZeroth:
      return EvaluateShapeFunctions0thDerivative(local_coords);
      break;
    case DerivativeOrder::kFirst:
      return EvaluateShapeFunctions1stDerivative(local_coords);
      break;
    case DerivativeOrder::kSecond:
      return EvaluateShapeFunctions2ndDerivative(local_coords);
      break;
    default:
      throw std::runtime_error("Invalid order of derivative");
  }
}

Eigen::MatrixXd GeometricEntity::GetNodesCoordinatesValues() const {
  Eigen::MatrixXd coords_values(number_of_nodes(), dimension_);
  for (size_t node_idx = 0; node_idx < number_of_nodes(); node_idx++) {
    const auto &node = nodes_[node_idx];
    const auto &coords = node->coords();
    for (size_t dimension_idx = 0; dimension_idx < dimension_;
         dimension_idx++) {
      coords_values(node_idx, dimension_idx) = coords.get(dimension_idx);
    }
  }
  return coords_values;
}

Eigen::MatrixXd GeometricEntity::EvaluateJacobian(
    const Coordinates &local_coords,
    const Eigen::MatrixXd &shape_functions_derivatives) const {
  const auto &nodes_coords_values = GetNodesCoordinatesValues();
  return shape_functions_derivatives * nodes_coords_values;
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
  for (size_t node_idx = 0; node_idx < number_of_nodes(); node_idx++) {
    const auto &node = nodes_[node_idx];
    const auto &coords = node->coords();
    for (size_t dimension_idx = 0; dimension_idx < dimension_;
         dimension_idx++) {
      xyz[dimension_idx] +=
          shape_functions_at_point(0, node_idx) * coords.get(dimension_idx);
    }
  }
  return Coordinates(xyz);
}

size_t GeometricEntity::GetNodeTag(size_t local_node_idx) const {
  return nodes_[local_node_idx]->tag();
}

Eigen::VectorXd GeometricEntity::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  std::string type_name = typeid(*this).name();
  throw std::logic_error("Normal is undefined for geometric entity of type " +
                         type_name);
}

Coordinates &GeometricEntity::GetCoordinatesOfNode(size_t node_idx) const {
  return nodes_[node_idx]->coords();
}

// Line entities

Line::Line(GeometricEntityType type, size_t dimensions,
           const std::vector<Node *> &nodes)
    : GeometricEntity(type, dimensions, nodes) {}

Eigen::VectorXd Line::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  if (dimensions() == 3) {
    throw std::runtime_error("Lines are not supported in 3D");
  }

  const auto &jacobian = EvaluateJacobian(local_coords);
  // [1x2] * [2x2] = [1x2] = [dx/dxi, dy/dxi]
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(2);
  normal(0) = jacobian(0, 1);
  normal(1) = -jacobian(0, 0);
  return normal;
}

double Line::EvaluateDifferential(const Coordinates &local_coords) const {
  const auto &normal = EvaluateNormalVector(local_coords);
  return normal.norm();
}

TwoNodeLine::TwoNodeLine(size_t dimensions, const std::vector<Node *> &nodes)
    : Line(GeometricEntityType::kTwoNodeLine, dimensions, nodes) {}

std::vector<Coordinates> TwoNodeLine::GetNodalParametricCoords() const {
  return {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
}

Eigen::MatrixXd TwoNodeLine::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 2);

  result(0, 0) = 0.5 * (1.0 - r);
  result(0, 1) = 0.5 * (1.0 + r);

  return result;
}

Eigen::MatrixXd TwoNodeLine::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 2);

  result(0, 0) = -0.5;
  result(0, 1) = 0.5;

  return result;
}

Eigen::MatrixXd TwoNodeLine::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  return Eigen::MatrixXd::Zero(1, 2);
}

// Quad entities

Quad::Quad(GeometricEntityType type, size_t dimensions,
           const std::vector<Node *> &nodes)
    : GeometricEntity(type, dimensions, nodes) {}

Eigen::VectorXd Quad::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  // [2x4] * [4x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  // For 2D we don't have dz
  const auto &jacobian = EvaluateJacobian(local_coords);
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);

  if (dimensions() == 3) {
    normal(2) =
        jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);
  }

  return normal;
}

double Quad::EvaluateDifferential(const Coordinates &local_coords) const {
  Eigen::VectorXd normal = EvaluateNormalVector(local_coords);
  return normal.norm();
}

FourNodeQuad::FourNodeQuad(size_t dimensions, const std::vector<Node *> &nodes)
    : Quad(GeometricEntityType::kFourNodeQuad, dimensions, nodes) {}

std::vector<Coordinates> FourNodeQuad::GetNodalParametricCoords() const {
  return {
      {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}};
}

Eigen::MatrixXd FourNodeQuad::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 4);

  result(0, 0) = 0.25 * (1 - r) * (1 - s);
  result(0, 1) = 0.25 * (1 + r) * (1 - s);
  result(0, 2) = 0.25 * (1 + r) * (1 + s);
  result(0, 3) = 0.25 * (1 - r) * (1 + s);

  return result;
}

Eigen::MatrixXd FourNodeQuad::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(2, 4);

  result(0, 0) = -0.25 * (1 - s);
  result(0, 1) = 0.25 * (1 - s);
  result(0, 2) = 0.25 * (1 + s);
  result(0, 3) = -0.25 * (1 + s);

  result(1, 0) = -0.25 * (1 - r);
  result(1, 1) = -0.25 * (1 + r);
  result(1, 2) = 0.25 * (1 + r);
  result(1, 3) = 0.25 * (1 - r);

  return result;
}

Eigen::MatrixXd FourNodeQuad::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 4);

  result(2, 0) = 0.25;
  result(2, 1) = -0.25;
  result(2, 2) = 0.25;
  result(2, 3) = -0.25;

  return result;
}

// Hex entities

Hex::Hex(GeometricEntityType type, size_t dimensions,
         const std::vector<Node *> &nodes)
    : GeometricEntity(type, dimensions, nodes) {}

double Hex::EvaluateDifferential(const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant();
}

EightNodeHex::EightNodeHex(const std::vector<Node *> &nodes)
    : Hex(GeometricEntityType::kEightNodeHex, 3, nodes) {}

std::vector<Coordinates> EightNodeHex::GetNodalParametricCoords() const {
  return {{-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0}, {1.0, 1.0, -1.0},
          {-1.0, 1.0, -1.0},  {-1.0, -1.0, 1.0}, {1.0, -1.0, 1.0},
          {1.0, 1.0, 1.0},    {-1.0, 1.0, 1.0}};
}

Eigen::MatrixXd EightNodeHex::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 8);

  result(0, 0) = 0.125 * (1 - r) * (1 - s) * (1 - t);
  result(0, 1) = 0.125 * (1 + r) * (1 - s) * (1 - t);
  result(0, 2) = 0.125 * (1 + r) * (1 + s) * (1 - t);
  result(0, 3) = 0.125 * (1 - r) * (1 + s) * (1 - t);
  result(0, 4) = 0.125 * (1 - r) * (1 - s) * (1 + t);
  result(0, 5) = 0.125 * (1 + r) * (1 - s) * (1 + t);
  result(0, 6) = 0.125 * (1 + r) * (1 + s) * (1 + t);
  result(0, 7) = 0.125 * (1 - r) * (1 + s) * (1 + t);

  return result;
}

Eigen::MatrixXd EightNodeHex::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 8);

  result(0, 0) = -0.125 * (1 - s) * (1 - t);
  result(0, 1) = 0.125 * (1 - s) * (1 - t);
  result(0, 2) = 0.125 * (1 + s) * (1 - t);
  result(0, 3) = -0.125 * (1 + s) * (1 - t);
  result(0, 4) = -0.125 * (1 - s) * (1 + t);
  result(0, 5) = 0.125 * (1 - s) * (1 + t);
  result(0, 6) = 0.125 * (1 + s) * (1 + t);
  result(0, 7) = -0.125 * (1 + s) * (1 + t);

  result(1, 0) = -0.125 * (1 - r) * (1 - t);
  result(1, 1) = -0.125 * (1 + r) * (1 - t);
  result(1, 2) = 0.125 * (1 + r) * (1 - t);
  result(1, 3) = 0.125 * (1 - r) * (1 - t);
  result(1, 4) = -0.125 * (1 - r) * (1 + t);
  result(1, 5) = -0.125 * (1 + r) * (1 + t);
  result(1, 6) = 0.125 * (1 + r) * (1 + t);
  result(1, 7) = 0.125 * (1 - r) * (1 + t);

  result(2, 0) = -0.125 * (1 - r) * (1 - s);
  result(2, 1) = -0.125 * (1 + r) * (1 - s);
  result(2, 2) = -0.125 * (1 + r) * (1 + s);
  result(2, 3) = -0.125 * (1 - r) * (1 + s);
  result(2, 4) = 0.125 * (1 - r) * (1 - s);
  result(2, 5) = 0.125 * (1 + r) * (1 - s);
  result(2, 6) = 0.125 * (1 + r) * (1 + s);
  result(2, 7) = 0.125 * (1 - r) * (1 + s);

  return result;
}

Eigen::MatrixXd EightNodeHex::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, 8);

  result(3, 0) = 0.125 * (1 - t);
  result(3, 1) = -0.125 * (1 - t);
  result(3, 2) = 0.125 * (1 - t);
  result(3, 3) = -0.125 * (1 - t);
  result(3, 4) = 0.125 * (1 + t);
  result(3, 5) = -0.125 * (1 + t);
  result(3, 6) = 0.125 * (1 + t);
  result(3, 7) = -0.125 * (1 + t);

  result(4, 0) = 0.125 * (1 - r);
  result(4, 1) = 0.125 * (1 + r);
  result(4, 2) = -0.125 * (1 + r);
  result(4, 3) = -0.125 * (1 - r);
  result(4, 4) = -0.125 * (1 - r);
  result(4, 5) = -0.125 * (1 + r);
  result(4, 6) = 0.125 * (1 + r);
  result(4, 7) = 0.125 * (1 - r);

  result(5, 0) = 0.125 * (1 - s);
  result(5, 1) = -0.125 * (1 - s);
  result(5, 2) = -0.125 * (1 + s);
  result(5, 3) = 0.125 * (1 + s);
  result(5, 4) = -0.125 * (1 - s);
  result(5, 5) = 0.125 * (1 - s);
  result(5, 6) = 0.125 * (1 + s);
  result(5, 7) = -0.125 * (1 + s);

  return result;
}

// Tria entities

Tria::Tria(GeometricEntityType type, size_t dimensions,
           const std::vector<Node *> &nodes)
    : GeometricEntity(type, dimensions, nodes) {}

Eigen::VectorXd Tria::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  // [2x3] * [3x3] = [2x3] = [dx/dxi, dy/dxi, dz/dxi
  //                          dx/deta, dy/deta, dz/deta]
  const auto &jacobian = EvaluateJacobian(local_coords);
  Eigen::VectorXd normal = Eigen::VectorXd::Zero(3);
  normal(0) = jacobian(0, 1) * jacobian(1, 2) - jacobian(0, 2) * jacobian(1, 1);
  normal(1) = jacobian(0, 2) * jacobian(1, 0) - jacobian(0, 0) * jacobian(1, 2);

  if (dimensions() == 3) {
    normal(2) =
        jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);
  }

  return normal;
}

double Tria::EvaluateDifferential(const Coordinates &local_coords) const {
  Eigen::VectorXd normal = EvaluateNormalVector(local_coords);
  return normal.norm() / 2.0;
}

ThreeNodeTria::ThreeNodeTria(size_t dimensions,
                             const std::vector<Node *> &nodes)
    : Tria(GeometricEntityType::kThreeNodeTria, dimensions, nodes) {}

std::vector<Coordinates> ThreeNodeTria::GetNodalParametricCoords() const {
  return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
}

Eigen::MatrixXd ThreeNodeTria::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 3);

  result(0, 0) = 1.0 - r - s;
  result(0, 1) = r;
  result(0, 2) = s;

  return result;
}

Eigen::MatrixXd ThreeNodeTria::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(2, 3);

  result(0, 0) = -1.0;
  result(0, 1) = 1.0;

  result(1, 0) = -1.0;
  result(1, 2) = 1.0;

  return result;
}

Eigen::MatrixXd ThreeNodeTria::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  return Eigen::MatrixXd::Zero(3, 3);
}

SixNodeTria::SixNodeTria(size_t dimensions, const std::vector<Node *> &nodes)
    : Tria(GeometricEntityType::kSixNodeTria, dimensions, nodes) {}

std::vector<Coordinates> SixNodeTria::GetNodalParametricCoords() const {
  return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0},
          {0.5, 0.0, 0.0}, {0.5, 0.5, 0.0}, {0.0, 0.5, 0.0}};
}

Eigen::MatrixXd SixNodeTria::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = 1 - r - s;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 6);

  result(0, 0) = (2.0 * t - 1.0) * t;
  result(0, 1) = (2.0 * r - 1.0) * r;
  result(0, 2) = (2.0 * s - 1.0) * s;
  result(0, 3) = 4.0 * t * r;
  result(0, 4) = 4.0 * r * s;
  result(0, 5) = 4.0 * s * t;

  return result;
}

Eigen::MatrixXd SixNodeTria::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = 1 - r - s;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(2, 6);

  result(0, 0) = -3.0 + 4.0 * (r + s);
  result(0, 1) = 4.0 * r - 1.0;
  result(0, 3) = 4.0 - 8.0 * r - 4.0 * s;
  result(0, 4) = 4.0 * s;
  result(0, 5) = -4.0 * s;

  result(1, 0) = -3.0 + 4.0 * (r + s);
  result(1, 2) = 4.0 * s - 1.0;
  result(1, 3) = -4.0 * r;
  result(1, 4) = 4.0 * r;
  result(1, 5) = 4.0 - 4.0 * r - 8.0 * s;

  return result;
}

Eigen::MatrixXd SixNodeTria::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = 1 - r - s;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 6);

  result(0, 0) = 4.0;
  result(0, 1) = 4.0;
  result(0, 3) = -8.0;

  result(1, 0) = 4.0;
  result(1, 2) = 4.0;
  result(1, 5) = -8.0;

  result(2, 0) = 4.0;
  result(2, 3) = -4.0;
  result(2, 4) = 4.0;
  result(2, 5) = -4.0;

  return result;
}

// Tetra entities

Tetra::Tetra(GeometricEntityType type, size_t dimensions,
             const std::vector<Node *> &nodes)
    : GeometricEntity(type, dimensions, nodes) {}

double Tetra::EvaluateDifferential(const Coordinates &local_coords) const {
  const auto &jacobian = EvaluateJacobian(local_coords);
  return jacobian.determinant() / 6.0;
}

FourNodeTetra::FourNodeTetra(const std::vector<Node *> &nodes)
    : Tetra(GeometricEntityType::kFourNodeTetra, 3, nodes) {}

std::vector<Coordinates> FourNodeTetra::GetNodalParametricCoords() const {
  return {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  };
}

Eigen::MatrixXd FourNodeTetra::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 4);

  result(0, 0) = 1.0 - r - s - t;
  result(0, 1) = r;
  result(0, 2) = s;
  result(0, 3) = t;

  return result;
}

Eigen::MatrixXd FourNodeTetra::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 4);

  result(0, 0) = -1.0;
  result(0, 1) = 1.0;

  result(1, 0) = -1.0;
  result(1, 2) = 1.0;

  result(2, 0) = -1.0;
  result(2, 3) = 1.0;

  return result;
}

Eigen::MatrixXd FourNodeTetra::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  return Eigen::MatrixXd::Zero(6, 4);
}

TenNodeTetra::TenNodeTetra(const std::vector<Node *> &nodes)
    : Tetra(GeometricEntityType::kTenNodeTetra, 3, nodes) {}

std::vector<Coordinates> TenNodeTetra::GetNodalParametricCoords() const {
  return {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
          {0.5, 0.0, 0.0}, {0.5, 0.5, 0.0}, {0.0, 0.5, 0.0}, {0.0, 0.0, 0.5},
          {0.0, 0.5, 0.5}, {0.5, 0.0, 0.5}};
}

Eigen::MatrixXd TenNodeTetra::EvaluateShapeFunctions0thDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  double u = 1.0 - r - s - t;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(1, 10);

  result(0, 0) = (2.0 * u - 1.0) * u;
  result(0, 1) = (2.0 * r - 1.0) * r;
  result(0, 2) = (2.0 * s - 1.0) * s;
  result(0, 3) = (2.0 * t - 1.0) * t;
  result(0, 4) = 4.0 * r * u;
  result(0, 5) = 4.0 * r * s;
  result(0, 6) = 4.0 * s * u;
  result(0, 7) = 4.0 * t * u;
  result(0, 8) = 4.0 * s * t;
  result(0, 9) = 4.0 * r * t;

  return result;
}

Eigen::MatrixXd TenNodeTetra::EvaluateShapeFunctions1stDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  double u = 1.0 - r - s - t;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 10);

  result(0, 0) = 4.0 * (r + s + t) - 3.0;
  result(0, 1) = 4.0 * r - 1.0;
  result(0, 4) = -8.0 * r - 4.0 * (s + t) + 4.0;
  result(0, 5) = 4.0 * s;
  result(0, 6) = -4.0 * s;
  result(0, 7) = -4.0 * t;
  result(0, 9) = 4.0 * t;

  result(1, 0) = 4.0 * (r + s + t) - 3.0;
  result(1, 2) = 4.0 * s - 1.0;
  result(1, 4) = -4.0 * r;
  result(1, 5) = 4.0 * r;
  result(1, 6) = -8.0 * s - 4.0 * (r + t) + 4.0;
  result(1, 7) = -4.0 * t;
  result(1, 8) = 4.0 * t;

  result(2, 0) = 4.0 * (r + s + t) - 3.0;
  result(2, 3) = 4.0 * t - 1.0;
  result(2, 4) = -4.0 * r;
  result(2, 6) = -4.0 * s;
  result(2, 7) = -8.0 * t - 4.0 * (r + s) + 4.0;
  result(2, 8) = 4.0 * s;
  result(2, 9) = 4.0 * r;

  return result;
}

Eigen::MatrixXd TenNodeTetra::EvaluateShapeFunctions2ndDerivative(
    const Coordinates &local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  double u = 1.0 - r - s - t;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, 10);

  result(0, 0) = 4.0;
  result(0, 1) = 4.0;
  result(0, 4) = -8.0;

  result(1, 0) = 4.0;
  result(1, 2) = 4.0;
  result(1, 6) = -8.0;

  result(2, 0) = 4.0;
  result(2, 3) = 4.0;
  result(2, 7) = -8.0;

  result(3, 0) = 4.0;
  result(3, 4) = -4.0;
  result(3, 5) = 4.0;
  result(3, 6) = -4.0;

  result(4, 0) = 4.0;
  result(4, 6) = -4.0;
  result(4, 7) = -4.0;
  result(4, 8) = 4.0;

  result(5, 0) = 4.0;
  result(5, 4) = -4.0;
  result(5, 7) = -4.0;
  result(5, 9) = 4.0;

  return result;
}

}  // namespace ffea
