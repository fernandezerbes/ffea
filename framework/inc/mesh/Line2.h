#ifndef FFEA_FRAMEWORK_INC_MESH_CURVEELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_CURVEELEMENT_H_

#include "./Element.h"

namespace ffea {

class Line2 : public Element
{
public:
  Line2(const std::vector<Node*> &nodes);
  virtual ~Line2();

  virtual double EvaluateJacobianDet(
    const Coordinates &local_coordinates) const override;

private:
  double ComputeLength() const;
  virtual std::vector<double> EvaluateShapeFunctions(
    const Coordinates &local_coordinates) const override;
};

} // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_CURVEELEMENT_H_