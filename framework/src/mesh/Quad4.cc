#include "../../inc/math/Utils.h"
#include "../../inc/mesh/Quad4.h"

namespace ffea {

Quad4::Quad4(const std::vector<Node*> &nodes) : Element(nodes) {}

Quad4::~Quad4() {}

//      eta
//       ^
//       |
// 3-----------2
// |     |     |
// |     |     |
// |     +---- | --> xi
// |           |
// |           |
// 0-----------1   

Eigen::MatrixXd Quad4::EvaluateJacobian(
    const Coordinates &local_coordinates) const {
  Eigen::MatrixXd jacobian(2, 2);
  const auto& shape_functions_derivatives = EvaluateShapeFunctionsDerivatives(
    local_coordinates);
  for (size_t node_index = 0; node_index < nodes_.size(); node_index++) {
    const auto& node = nodes_[node_index];
    const auto& coords = node->coordinates();
    jacobian(0, 0) += coords.get(0) * shape_functions_derivatives(0, node_index);
    jacobian(0, 1) += coords.get(1) * shape_functions_derivatives(0, node_index);
    jacobian(1, 0) += coords.get(0) * shape_functions_derivatives(1, node_index);
    jacobian(1, 1) += coords.get(1) * shape_functions_derivatives(1, node_index);
  }
  return jacobian;
}

Eigen::MatrixXd Quad4::EvaluateShapeFunctions(
    const Coordinates &local_coordinates) const {
  double xi = local_coordinates.get(0);
  double eta = local_coordinates.get(1);
  Eigen::MatrixXd shape_functions(1, 4);
  shape_functions(0, 0) = 0.25 * (1 - xi) * (1 - eta);
  shape_functions(0, 1) = 0.25 * (1 + xi) * (1 - eta);
  shape_functions(0, 2) = 0.25 * (1 + xi) * (1 + eta);
  shape_functions(0, 3) = 0.25 * (1 - xi) * (1 + eta);
  return shape_functions;
}

Eigen::MatrixXd Quad4::EvaluateShapeFunctionsDerivatives(
    const Coordinates &local_coordinates) const {
  double xi = local_coordinates.get(0);
  double eta = local_coordinates.get(1);
  Eigen::MatrixXd shape_functions_derivatives(2, 4);
  shape_functions_derivatives(0, 0) = -0.25 * (1 - eta);
  shape_functions_derivatives(0, 1) = 0.25 * (1 - eta);
  shape_functions_derivatives(0, 2) = 0.25 * (1 + eta);
  shape_functions_derivatives(0, 3) = -0.25 * (1 + eta);
  shape_functions_derivatives(1, 0) = -0.25 * (1 - xi);
  shape_functions_derivatives(1, 1) = -0.25 * (1 + xi);
  shape_functions_derivatives(1, 2) = 0.25 * (1 + xi);
  shape_functions_derivatives(1, 3) = 0.25 * (1 - xi);
  return shape_functions_derivatives;
}

} // namespace ffea