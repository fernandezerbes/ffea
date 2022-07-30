#include "../../inc/mesh/Line2.h"

#include "../../inc/math/Utils.h"

namespace ffea {

Line2::Line2(const std::vector<Node *> &nodes) : Element(nodes) {}

Line2::~Line2() {}

Eigen::MatrixXd Line2::EvaluateJacobian(
    const Coordinates &local_coordinates) const {
  Eigen::MatrixXd jacobian(1, 1);
  jacobian(0, 0) = ComputeLength();
  return jacobian;
}

double Line2::ComputeLength() const {
  const auto &first_node_coordinates = nodes_[0]->coordinates();
  const auto &second_node_coordinates = nodes_[1]->coordinates();

  double length = utilities::DistanceBetweenPoints(
      first_node_coordinates.get(), second_node_coordinates.get());
  return length;
}

Eigen::MatrixXd Line2::EvaluateShapeFunctions(
    const Coordinates &local_coordinates) const {
  double xi = local_coordinates.get(0);
  Eigen::MatrixXd shape_functions(1, 2);
  shape_functions(0, 0) = 0.5 * (1.0 - xi);
  shape_functions(0, 1) = 0.5 * (1.0 + xi);
  return shape_functions;
}

Eigen::MatrixXd Line2::EvaluateShapeFunctionsDerivatives(
    const Coordinates &local_coordinates) const {
  Eigen::MatrixXd shape_functions_derivatives(1, 2);
  shape_functions_derivatives(0, 0) = 1.0;
  shape_functions_derivatives(0, 1) = -1.0;
  return shape_functions_derivatives;
}

}  // namespace ffea