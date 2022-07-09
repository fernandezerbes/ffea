#ifndef FFEA_FRAMEWORK_MATH_MATRIX_H_
#define FFEA_FRAMEWORK_MATH_MATRIX_H_

#include <cstddef>
#include <iostream>

namespace ffea {

template<typename T>
class Matrix
{
 public:
  Matrix(size_t number_of_rows, size_t number_of_columns);
  virtual ~Matrix();
  
  virtual T &operator()(size_t i, size_t j) = 0;

  size_t number_of_rows() const;
  size_t number_of_columns() const;

 private:
  size_t number_of_rows_;
  size_t number_of_columns_;
};

template<typename T>
Matrix<T>::Matrix(size_t number_of_rows, size_t number_of_columns)
  : number_of_rows_(number_of_rows),
    number_of_columns_(number_of_columns)
  {}

template<typename T>
Matrix<T>::~Matrix() {}

template<typename T>
size_t Matrix<T>::number_of_rows() const {
  return number_of_rows_;
}

template<typename T>
size_t Matrix<T>::number_of_columns() const {
  return number_of_columns_;
}

} // namespace ffea

#endif // FFEA_FRAMEWORK_MATH_MATRIX_H_
