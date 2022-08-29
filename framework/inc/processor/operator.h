#ifndef FFEA_FRAMEWORK_PROCESSOR_OPERATOR_H_
#define FFEA_FRAMEWORK_PROCESSOR_OPERATOR_H_

#include <eigen3/Eigen/Dense>

namespace ffea {

class DifferentialOperator {
 public:
  DifferentialOperator(size_t physical_dimension);
  virtual const Eigen::MatrixXd Compute(
      const Eigen::MatrixXd &shape_function_derivatives) const = 0;

 protected:
  size_t physical_dimension_;
};

class StrainDisplacementOperator2D : public DifferentialOperator {
 public:
  StrainDisplacementOperator2D();
  virtual const Eigen::MatrixXd Compute(
      const Eigen::MatrixXd &shape_function_derivatives) const override;
};

class StrainDisplacementOperator3D : public DifferentialOperator {
 public:
  StrainDisplacementOperator3D();
  virtual const Eigen::MatrixXd Compute(
      const Eigen::MatrixXd &shape_function_derivatives) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_PROCESSOR_OPERATOR_H_