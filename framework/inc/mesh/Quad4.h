#ifndef FFEA_FRAMEWORK_INC_MESH_QUAD4_H_
#define FFEA_FRAMEWORK_INC_MESH_QUAD4_H_

#include "./Element.h"

namespace ffea {

class Quad4 : public Element
{
 public:
  Quad4(const std::vector<Node*> &nodes);
  virtual ~Quad4();

  virtual Eigen::MatrixXd EvaluateJacobian(
    const Coordinates &local_coordinates) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions(
    const Coordinates &local_coordinates) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctionsDerivatives(
    const Coordinates &local_coordinates) const override;
};

} // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_QUAD4_H_