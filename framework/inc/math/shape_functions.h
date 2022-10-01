#ifndef FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_
#define FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_

#include <eigen3/Eigen/Dense>

#include "../geometry/coordinates.h"

namespace ffea {

enum class DerivativeOrder { kZeroth, kFirst, kSecond };

class ShapeFunctions {
 public:
  Eigen::MatrixXd Evaluate(const Coordinates& local_coords,
                           DerivativeOrder derivative_order) const;

 private:
  virtual Eigen::MatrixXd Evaluate(const Coordinates& local_coords) const = 0;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const = 0;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const = 0;
};

class TwoNodeLineShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};

class ThreeNodeTriaShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};

class FourNodeQuadShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};

class FourNodeTetraShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};
class EightNodeHexShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};

class SixNodeTriaShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};

class TenNodeTetraShapeFunctions : public ShapeFunctions {
 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& local_coords) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& local_coords) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_