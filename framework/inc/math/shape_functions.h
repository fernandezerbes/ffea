#ifndef FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_
#define FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_

#include <eigen3/Eigen/Dense>
#include <vector>

namespace ffea {

enum class DerivativeOrder { kZeroth, kFirst, kSecond };

class ShapeFunctions {
 public:
  Eigen::MatrixXd Evaluate(const std::vector<double>& coordinates,
                           DerivativeOrder derivative_order) const;

  virtual ~ShapeFunctions();

 private:
  virtual Eigen::MatrixXd Evaluate(const std::vector<double>& coordinates) const = 0;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const std::vector<double>& coordinates) const = 0;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const std::vector<double>& coordinates) const = 0;
};

class Linear1DShapeFunctions : public ShapeFunctions {
 public:
  virtual ~Linear1DShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const std::vector<double>& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const std::vector<double>& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const std::vector<double>& coordinates) const override;
};

class Linear2DShapeFunctions : public ShapeFunctions {
 public:
  virtual ~Linear2DShapeFunctions() override;

 private:
  virtual Eigen::MatrixXd Evaluate(
      const std::vector<double>& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate1stDerivative(
      const std::vector<double>& coordinates) const override;
  virtual Eigen::MatrixXd Evaluate2ndDerivative(
      const std::vector<double>& coordinates) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MATH_SHAPE_FUNCTIONS_H_