#include "../../inc/math/shape_functions.h"

#include <stdexcept>

namespace ffea {

Eigen::MatrixXd ShapeFunctions::Evaluate(
    const std::vector<double>& coordinates,
    DerivativeOrder derivative_order) const {
  switch (derivative_order) {
    case DerivativeOrder::kZeroth:
      return Evaluate(coordinates);
      break;
    case DerivativeOrder::kFirst:
      return Evaluate1stDerivative(coordinates);
      break;
    case DerivativeOrder::kSecond:
      return Evaluate2ndDerivative(coordinates);
      break;
    default:
      throw std::runtime_error("Invalid order of derivative");
  }
}

ShapeFunctions::~ShapeFunctions() {}

Linear1DShapeFunctions::~Linear1DShapeFunctions() {}

Eigen::MatrixXd Linear1DShapeFunctions::Evaluate(
    const std::vector<double>& coordinates) const {
  double xi = coordinates[0];
  Eigen::MatrixXd result(1, 2);
  result(0, 0) = 0.5 * (1.0 - xi);
  result(0, 1) = 0.5 * (1.0 + xi);
  return result;
}

Eigen::MatrixXd Linear1DShapeFunctions::Evaluate1stDerivative(
    const std::vector<double>& coordinates) const {
  Eigen::MatrixXd result(1, 2);
  result(0, 0) = 1.0;
  result(0, 1) = -1.0;
  return result;
}

Eigen::MatrixXd Linear1DShapeFunctions::Evaluate2ndDerivative(
    const std::vector<double>& coordinates) const {
  Eigen::MatrixXd result(1, 2);
  result(0, 0) = 0.0;
  result(0, 1) = 0.0;
  return result;
}

Linear2DShapeFunctions::~Linear2DShapeFunctions() {}

Eigen::MatrixXd Linear2DShapeFunctions::Evaluate(
    const std::vector<double>& coordinates) const {
  double xi = coordinates[0];
  double eta = coordinates[1];
  Eigen::MatrixXd result(1, 4);
  result(0, 0) = 0.25 * (1 - xi) * (1 - eta);
  result(0, 1) = 0.25 * (1 + xi) * (1 - eta);
  result(0, 2) = 0.25 * (1 + xi) * (1 + eta);
  result(0, 3) = 0.25 * (1 - xi) * (1 + eta);
  return result;
}

Eigen::MatrixXd Linear2DShapeFunctions::Evaluate1stDerivative(
    const std::vector<double>& coordinates) const {
  /*
    [dN1/dxi   dN2/dxi    dN3/dxi   dN4/dxi,
     dN1/deta  dN2/deta   dN3/deta  dN4/deta]
  */
  double xi = coordinates[0];
  double eta = coordinates[1];
  Eigen::MatrixXd result(2, 4);
  result(0, 0) = -0.25 * (1 - eta);
  result(0, 1) = 0.25 * (1 - eta);
  result(0, 2) = 0.25 * (1 + eta);
  result(0, 3) = -0.25 * (1 + eta);

  result(1, 0) = -0.25 * (1 - xi);
  result(1, 1) = -0.25 * (1 + xi);
  result(1, 2) = 0.25 * (1 + xi);
  result(1, 3) = 0.25 * (1 - xi);
  return result;
}

Eigen::MatrixXd Linear2DShapeFunctions::Evaluate2ndDerivative(
    const std::vector<double>& coordinates) const {
  /*
    [d^2N1/dxi^2      d^2N2/dxi^2     d^2N3/dxi^2     d^2N4/dxi^2,
     d^2N1/deta^2     d^2N2/deta^2    d^2N3/deta^2    d^2N4/deta^2,
     d^2N1/dxideta    d^2N2/dxideta   d^2N3/dxideta   d^2N4/dxideta]
  */
  double xi = coordinates[0];
  double eta = coordinates[1];
  Eigen::MatrixXd result(3, 4);
  result(0, 0) = 0.0;
  result(0, 1) = 0.0;
  result(0, 2) = 0.0;
  result(0, 3) = 0.0;

  result(1, 0) = 0.0;
  result(1, 1) = 0.0;
  result(1, 2) = 0.0;
  result(1, 3) = 0.0;

  result(2, 0) = 0.25;
  result(2, 1) = -0.25;
  result(2, 2) = 0.25;
  result(2, 3) = -0.25;
  return result;
}

}  // namespace ffea
