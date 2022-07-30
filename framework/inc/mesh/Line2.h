#ifndef FFEA_FRAMEWORK_INC_MESH_LINE2_H_
#define FFEA_FRAMEWORK_INC_MESH_LINE2_H_

#include "./Element.h"

namespace ffea {

class Line2 : public Element {
 public:
  Line2(const std::vector<Node *> &nodes);
  virtual ~Line2();

  virtual Eigen::MatrixXd EvaluateJacobian(
      const Coordinates &local_coordinates) const override;
  double ComputeLength() const;
  virtual Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coordinates) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctionsDerivatives(
      const Coordinates &local_coordinates) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_LINE2_H_