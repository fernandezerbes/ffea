#ifndef FFEA_FRAMEWORK_MATH_CUSTOMDENSEMATRIX_H_
#define FFEA_FRAMEWORK_MATH_CUSTOMDENSEMATRIX_H_

#include <vector>

#include "./Matrix.h"

namespace ffea {

class CustomDenseMatrix : public Matrix
{
 public:
  CustomDenseMatrix(size_t number_of_rows, size_t number_of_columns);
  virtual ~CustomDenseMatrix() override;

  virtual double &operator()(size_t i, size_t j) override;

 private:
  std::vector<double> data_;
};

CustomDenseMatrix::CustomDenseMatrix(
    size_t number_of_rows, size_t number_of_columns)
  : Matrix(number_of_rows, number_of_columns),
    data_(number_of_rows * number_of_columns)
  {}

CustomDenseMatrix::~CustomDenseMatrix() {}

double &CustomDenseMatrix::operator()(size_t i, size_t j) {
  return data_[i * number_of_rows() + j];
}

} // namespace ffea

#endif // FFEA_FRAMEWORK_MATH_CUSTOMDENSEMATRIX_H_
