#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <vector>

#include "./Coordinates.h"
#include "./DegreeOfFreedom.h"
#include "./Node.h"

namespace ffea
{

class Element {
 public:
  Element(const std::vector<Node*> &nodes);
  virtual ~Element();

  std::vector<Node*> &nodes();
  std::vector<Coordinates*> node_coordinates() const;
  std::vector<DegreeOfFreedom*> dofs() const;
  virtual double EvaluateJacobianDet(
    const Coordinates &local_coordinates) const = 0;

 protected:
  std::vector<Node*> nodes_;

 private:
  virtual std::vector<double> EvaluateShapeFunctions(
    const Coordinates &local_coordinates) const = 0;
};

}

#endif // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
