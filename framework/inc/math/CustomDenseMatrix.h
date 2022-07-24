// #ifndef FFEA_FRAMEWORK_MATH_CUSTOMDENSEMATRIX_H_
// #define FFEA_FRAMEWORK_MATH_CUSTOMDENSEMATRIX_H_

// #include <vector>

// #include "./Matrix.h"

// namespace ffea {

// template<typename T>
// class CustomDenseMatrix : public Matrix<T>
// {
//  public:
//   CustomDenseMatrix(size_t number_of_rows, size_t number_of_columns);
//   virtual ~CustomDenseMatrix() override;

//   virtual T &operator()(size_t i, size_t j) override;
//   virtual const T &operator()(size_t i, size_t j) override;

//  private:
//   std::vector<T> data_;
// };

// template<typename T>
// CustomDenseMatrix<T>::CustomDenseMatrix(
//     size_t number_of_rows, size_t number_of_columns)
//   : Matrix<T>(number_of_rows, number_of_columns),
//     data_(number_of_rows * number_of_columns)
//   {}

// template<typename T>
// CustomDenseMatrix<T>::~CustomDenseMatrix() {}

// template<typename T>
// T &CustomDenseMatrix<T>::operator()(size_t i, size_t j) {
//   return data_[i * Matrix<T>::number_of_rows() + j];
// }

// const T &CustomDenseMatrix<T>::operator()(size_t i, size_t j) {
//   return data_[i * Matrix<T>::number_of_rows() + j];
// }

// } // namespace ffea

// #endif // FFEA_FRAMEWORK_MATH_CUSTOMDENSEMATRIX_H_
