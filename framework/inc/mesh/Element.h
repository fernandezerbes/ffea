#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "../math/shape_functions.h"
#include "./Coordinates.h"
#include "./DegreeOfFreedom.h"
#include "./Node.h"

namespace ffea {

class Element {
 public:
  Element(const std::vector<Node *> &nodes,
          std::unique_ptr<ShapeFunctions> shape_functions);
  virtual ~Element();

  std::vector<Node *> &nodes();
  std::vector<Coordinates *> node_coordinates() const;
  std::vector<DegreeOfFreedom *> dofs() const;
  size_t number_of_nodes() const;
  size_t number_of_dofs() const;
  virtual Eigen::MatrixXd EvaluateJacobian(
      const Coordinates &local_coordinates) const = 0;
  virtual Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coordinates) const = 0;
  virtual Eigen::MatrixXd EvaluateShapeFunctionsDerivatives(
      const Coordinates &local_coordinates) const = 0;

 protected:
  std::vector<Node *> nodes_;
  std::unique_ptr<ShapeFunctions> shape_functions_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
