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

  result(0, 0) = -0.5;
  result(0, 1) = 0.5;

  return result;
}

Eigen::MatrixXd Linear1DShapeFunctions::Evaluate2ndDerivative(
    const std::vector<double>& coordinates) const {
  return Eigen::MatrixXd::Zero(1, 2);
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
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 4);

  result(2, 0) = 0.25;
  result(2, 1) = -0.25;
  result(2, 2) = 0.25;
  result(2, 3) = -0.25;

  return result;
}

Linear3DShapeFunctions::~Linear3DShapeFunctions() {}

Eigen::MatrixXd Linear3DShapeFunctions::Evaluate(
    const std::vector<double>& coordinates) const {
  double xi = coordinates[0];
  double eta = coordinates[1];
  double zeta = coordinates[2];
  Eigen::MatrixXd result(1, 8);

  result(0, 0) = 0.125 * (1 - xi) * (1 - eta) * (1 - zeta);
  result(0, 1) = 0.125 * (1 + xi) * (1 - eta) * (1 - zeta);
  result(0, 2) = 0.125 * (1 + xi) * (1 + eta) * (1 - zeta);
  result(0, 3) = 0.125 * (1 - xi) * (1 + eta) * (1 - zeta);
  result(0, 4) = 0.125 * (1 - xi) * (1 - eta) * (1 + zeta);
  result(0, 5) = 0.125 * (1 + xi) * (1 - eta) * (1 + zeta);
  result(0, 6) = 0.125 * (1 + xi) * (1 + eta) * (1 + zeta);
  result(0, 7) = 0.125 * (1 - xi) * (1 + eta) * (1 + zeta);
  
  return result;
}

Eigen::MatrixXd Linear3DShapeFunctions::Evaluate1stDerivative(
    const std::vector<double>& coordinates) const {
  /*
    [dN1/dxi   dN2/dxi    dN3/dxi   dN4/dxi   dN5/dxi   dN6/dxi   dN7/dxi   dN8/dxi,
     dN1/deta  dN2/deta   dN3/deta  dN4/deta  dN5/deta  dN6/deta  dN7/deta  dN8/deta,
     dN1/dzeta dN2/dzeta  dN3/zdeta dN4/dzeta dN5/dzeta dN6/dzeta dN7/dzeta dN8/dzeta]
  */
  double xi = coordinates[0];
  double eta = coordinates[1];
  double zeta = coordinates[2];
  Eigen::MatrixXd result(3, 8);

  result(0, 0) = -0.125 * (1 - eta) * (1 - zeta);
  result(0, 1) = 0.125 * (1 - eta) * (1 - zeta);
  result(0, 2) = 0.125 * (1 + eta) * (1 - zeta);
  result(0, 3) = -0.125 * (1 + eta) * (1 - zeta);
  result(0, 4) = -0.125 * (1 - eta) * (1 + zeta);
  result(0, 5) = 0.125 * (1 - eta) * (1 + zeta);
  result(0, 6) = 0.125 * (1 + eta) * (1 + zeta);
  result(0, 7) = -0.125 * (1 + eta) * (1 + zeta);

  result(1, 0) = -0.125 * (1 - xi) * (1 - zeta);
  result(1, 1) = -0.125 * (1 + xi) * (1 - zeta);
  result(1, 2) = 0.125 * (1 + xi) * (1 - zeta);
  result(1, 3) = 0.125 * (1 - xi) * (1 - zeta);
  result(1, 4) = -0.125 * (1 - xi) * (1 + zeta);
  result(1, 5) = -0.125 * (1 + xi) * (1 + zeta);
  result(1, 6) = 0.125 * (1 + xi) * (1 + zeta);
  result(1, 7) = 0.125 * (1 - xi) * (1 + zeta);

  result(2, 0) = -0.125 * (1 - xi) * (1 - eta);
  result(2, 1) = -0.125 * (1 + xi) * (1 - eta);
  result(2, 2) = -0.125 * (1 + xi) * (1 + eta);
  result(2, 3) = -0.125 * (1 - xi) * (1 + eta);
  result(2, 4) = 0.125 * (1 - xi) * (1 - eta);
  result(2, 5) = 0.125 * (1 + xi) * (1 - eta);
  result(2, 6) = 0.125 * (1 + xi) * (1 + eta);
  result(2, 7) = 0.125 * (1 - xi) * (1 + eta);

  return result;
}

Eigen::MatrixXd Linear3DShapeFunctions::Evaluate2ndDerivative(
    const std::vector<double>& coordinates) const {
  /*
    [d^2N1/dxi^2      d^2N2/dxi^2     d^2N3/dxi^2     d^2N4/dxi^2       d^2N5/dxi^2      d^2N6/dxi^2     d^2N7/dxi^2     d^2N8/dxi^2,
     d^2N1/deta^2     d^2N2/deta^2    d^2N3/deta^2    d^2N4/deta^2      d^2N5/deta^2     d^2N6/deta^2    d^2N7/deta^2    d^2N8/deta^2,
     d^2N1/dzeta^2    d^2N2/dzeta^2   d^2N3/dzeta^2   d^2N4/dzeta^2     d^2N5/dzeta^2    d^2N6/dzeta^2   d^2N7/dzeta^2   d^2N8/dzeta^2,
     d^2N1/dxideta    d^2N2/dxideta   d^2N3/dxideta   d^2N4/dxideta     d^2N5/dxideta    d^2N6/dxideta   d^2N7/dxideta   d^2N8/dxideta,
     d^2N1/detadzeta  d^2N2/detadzeta d^2N3/detadzeta d^2N4/detadzeta   d^2N5/detadzeta  d^2N6/detadzeta d^2N7/detadzeta d^2N8/detadzeta,
     d^2N1/dzetadxi   d^2N2/dzetadxi  d^2N3/dzetadxi  d^2N4/dzetadxi    d^2N5/dzetadxi   d^2N6/dzetadxi  d^2N7/dzetadxi  d^2N8/dzetadxi]
  */

  double xi = coordinates[0];
  double eta = coordinates[1];
  double zeta = coordinates[2];
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, 8);
  
  result(3, 0) = 0.125 * (1 - zeta);
  result(3, 1) = -0.125 * (1 - zeta);
  result(3, 2) = 0.125 * (1 - zeta);
  result(3, 3) = -0.125 * (1 - zeta);
  result(3, 4) = 0.125 * (1 + zeta);
  result(3, 5) = -0.125 * (1 + zeta);
  result(3, 6) = 0.125 * (1 + zeta);
  result(3, 7) = -0.125 * (1 + zeta);

  result(4, 0) = 0.125 * (1 - xi);
  result(4, 1) = 0.125 * (1 + xi);
  result(4, 2) = -0.125 * (1 + xi);
  result(4, 3) = -0.125 * (1 - xi);
  result(4, 4) = -0.125 * (1 - xi);
  result(4, 5) = -0.125 * (1 + xi);
  result(4, 6) = 0.125 * (1 + xi);
  result(4, 7) = 0.125 * (1 - xi);

  result(5, 0) = 0.125 * (1 - eta);
  result(5, 1) = -0.125 * (1 - eta);
  result(5, 2) = -0.125 * (1 + eta);
  result(5, 3) = 0.125 * (1 + eta);
  result(5, 4) = -0.125 * (1 - eta);
  result(5, 5) = 0.125 * (1 - eta);
  result(5, 6) = 0.125 * (1 + eta);
  result(5, 7) = -0.125 * (1 + eta);

  return result;
}

}  // namespace ffea
