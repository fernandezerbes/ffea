#ifndef FFEA_FRAMEWORK_PROCESSOR_OPERATOR_H_
#define FFEA_FRAMEWORK_PROCESSOR_OPERATOR_H_

#include "../math/CustomDenseMatrix.h"

namespace ffea
{

class StrainDisplacementOperator
{
 public:
  StrainDisplacementOperator();
  ~StrainDisplacementOperator();

  CustomDenseMatrix<double> get(
    const CustomDenseMatrix<double> &shape_function_derivatives);
};

} // namespace ffea

#endif // FFEA_FRAMEWORK_PROCESSOR_OPERATOR_H_