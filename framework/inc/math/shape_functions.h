#ifndef FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_
#define FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_

#include <eigen3/Eigen/Dense>

#include "../geometry/coordinates.h"

namespace ffea {

enum class DerivativeOrder { kZeroth, kFirst, kSecond };

class ShapeFunctions {
 public:
  Eigen::MatrixXd Evaluate(const Coordinates& coordinates,
                           DerivativeOrder derivative_order) const;

  virtual ~ShapeFunctions();

 private:
  virtual Eigen::MatrixXd Evaluate(const Coordinates& coordinates) const = 0;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const = 0;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const = 0;
};

class TwoNodeLineShapeFunctions : public ShapeFunctions {
 public:
  virtual ~TwoNodeLineShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};

class ThreeNodeTriaShapeFunctions : public ShapeFunctions {
 public:
  virtual ~ThreeNodeTriaShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};

class FourNodeQuadShapeFunctions : public ShapeFunctions {
 public:
  virtual ~FourNodeQuadShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};

class FourNodeTetraShapeFunctions : public ShapeFunctions {
 public:
  virtual ~FourNodeTetraShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};
class EightNodeHexShapeFunctions : public ShapeFunctions {
 public:
  virtual ~EightNodeHexShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};

class SixNodeTriaShapeFunctions : public ShapeFunctions {
 public:
  virtual ~SixNodeTriaShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};

class TenNodeTetraShapeFunctions : public ShapeFunctions {
 public:
  virtual ~TenNodeTetraShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const Coordinates& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const Coordinates& coordinates) const override;
};


}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_