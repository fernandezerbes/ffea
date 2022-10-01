#include "../../inc/math/shape_functions.h"

#include <stdexcept>

namespace ffea {

Eigen::MatrixXd ShapeFunctions::Evaluate(
    const Coordinates& local_coords, DerivativeOrder derivative_order) const {
  switch (derivative_order) {
    case DerivativeOrder::kZeroth:
      return Evaluate(local_coords);
      break;
    case DerivativeOrder::kFirst:
      return Evaluate1stDerivative(local_coords);
      break;
    case DerivativeOrder::kSecond:
      return Evaluate2ndDerivative(local_coords);
      break;
    default:
      throw std::runtime_error("Invalid order of derivative");
  }
}

Eigen::MatrixXd TwoNodeLineShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  Eigen::MatrixXd result(1, 2);

  result(0, 0) = 0.5 * (1.0 - r);
  result(0, 1) = 0.5 * (1.0 + r);

  return result;
}

Eigen::MatrixXd TwoNodeLineShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
  Eigen::MatrixXd result(1, 2);

  result(0, 0) = -0.5;
  result(0, 1) = 0.5;

  return result;
}

Eigen::MatrixXd TwoNodeLineShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
  return Eigen::MatrixXd::Zero(1, 2);
}

Eigen::MatrixXd ThreeNodeTriaShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result(1, 3);

  result(0, 0) = 1.0 - r - s;
  result(0, 1) = r;
  result(0, 2) = s;

  return result;
}

Eigen::MatrixXd ThreeNodeTriaShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(2, 3);

  result(0, 0) = -1.0;
  result(0, 1) = 1.0;

  result(1, 0) = -1.0;
  result(1, 2) = 1.0;

  return result;
}

Eigen::MatrixXd ThreeNodeTriaShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
  return Eigen::MatrixXd::Zero(3, 3);
}

Eigen::MatrixXd FourNodeQuadShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result(1, 4);

  result(0, 0) = 0.25 * (1 - r) * (1 - s);
  result(0, 1) = 0.25 * (1 + r) * (1 - s);
  result(0, 2) = 0.25 * (1 + r) * (1 + s);
  result(0, 3) = 0.25 * (1 - r) * (1 + s);

  return result;
}

Eigen::MatrixXd FourNodeQuadShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result(2, 4);

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

Eigen::MatrixXd FourNodeQuadShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 4);

  result(2, 0) = 0.25;
  result(2, 1) = -0.25;
  result(2, 2) = 0.25;
  result(2, 3) = -0.25;

  return result;
}

Eigen::MatrixXd FourNodeTetraShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result(1, 4);

  result(0, 0) = 1.0 - r - s - t;
  result(0, 1) = r;
  result(0, 2) = s;
  result(0, 3) = t;

  return result;
}

Eigen::MatrixXd FourNodeTetraShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result(3, 4);

  result(0, 0) = -1.0;
  result(0, 1) = 1.0;

  result(1, 0) = -1.0;
  result(1, 2) = 1.0;

  result(2, 0) = -1.0;
  result(2, 3) = 1.0;

  return result;
}

Eigen::MatrixXd FourNodeTetraShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
  return Eigen::MatrixXd::Zero(6, 4);
}

Eigen::MatrixXd EightNodeHexShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result(1, 8);

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

Eigen::MatrixXd EightNodeHexShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  Eigen::MatrixXd result(3, 8);

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

Eigen::MatrixXd EightNodeHexShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
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

Eigen::MatrixXd SixNodeTriaShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = 1 - r - s;
  Eigen::MatrixXd result(1, 6);

  result(0, 0) = (2.0 * t - 1.0) * t;
  result(0, 1) = (2.0 * r - 1.0) * r;
  result(0, 2) = (2.0 * s - 1.0) * s;
  result(0, 3) = 4.0 * t * r;
  result(0, 4) = 4.0 * r * s;
  result(0, 5) = 4.0 * s * t;

  return result;
}

Eigen::MatrixXd SixNodeTriaShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
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

Eigen::MatrixXd SixNodeTriaShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
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

Eigen::MatrixXd TenNodeTetraShapeFunctions::Evaluate(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  double u = 1.0 - r - s - t;
  Eigen::MatrixXd result(1, 10);

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

Eigen::MatrixXd TenNodeTetraShapeFunctions::Evaluate1stDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  double u = 1.0 - r - s - t;
  Eigen::MatrixXd result(3, 10);

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

Eigen::MatrixXd TenNodeTetraShapeFunctions::Evaluate2ndDerivative(
    const Coordinates& local_coords) const {
  double r = local_coords.get(0);
  double s = local_coords.get(1);
  double t = local_coords.get(2);
  double u = 1.0 - r - s - t;
  Eigen::MatrixXd result(6, 10);

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
