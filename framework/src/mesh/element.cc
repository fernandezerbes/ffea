#include "../../inc/mesh/element.h"

namespace ffea {

Element::Element(GeometricEntity &geometric_entity,
                 const std::vector<DegreeOfFreedom *> &dofs,
                 const IntegrationPointsGroup &integration_points)
    : geometric_entity_(geometric_entity),
      dofs_(dofs),
      integration_points_(integration_points) {}

std::vector<size_t> Element::GetLocalToGlobalDofIndicesMap() const {
  std::vector<size_t> indices_map;
  indices_map.reserve(GetNumberOfDofs());
  for (const auto &dof : dofs_) {
    indices_map.push_back(dof->local_id());
  }
  return indices_map;
}

size_t Element::GetNumberOfDofs() const { return dofs_.size(); }

size_t Element::GetNumberOfNodes() const {
  return geometric_entity_.GetNumberOfNodes();
}

size_t Element::GetNumberOfDofsPerNode() const {
  return GetNumberOfDofs() / GetNumberOfNodes();
}

Coordinates &Element::GetCoordinatesOfNode(size_t node_index) const {
  return geometric_entity_.GetCoordinatesOfNode(node_index);
}

Eigen::VectorXd Element::GetSolutionFromDofs(size_t component_index) const {
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(GetNumberOfNodes());

  size_t dof_index = component_index;
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &dof = dofs_[dof_index];
    solution(node_index) = dof->value();
    dof_index += 2;
  }

  return solution;
}

const std::vector<Node *> &Element::nodes() const {
  return geometric_entity_.nodes();
}

Eigen::MatrixXd Element::EvaluateJacobian(
    const Coordinates &local_coords,
    const Eigen::MatrixXd &shape_functions_derivatives) const {
  return geometric_entity_.EvaluateJacobian(local_coords,
                                            shape_functions_derivatives);
}

Eigen::MatrixXd Element::EvaluateShapeFunctions(
    const Coordinates &local_coords, DerivativeOrder derivative_order) const {
  return geometric_entity_.EvaluateShapeFunctions(local_coords,
                                                  derivative_order);
}

Eigen::VectorXd Element::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  return geometric_entity_.EvaluateNormalVector(local_coords);
}

double Element::EvaluateDifferential(const Coordinates &local_coords) const {
  return geometric_entity_.EvaluateDifferential(local_coords);
}

Coordinates Element::MapLocalToGlobal(const Coordinates &local_coords) const {
  return geometric_entity_.MapLocalToGlobal(local_coords);
}

Coordinates Element::MapLocalToGlobal(
    const Eigen::MatrixXd &shape_functions_at_point) const {
  return geometric_entity_.MapLocalToGlobal(shape_functions_at_point);
}

const IntegrationPointsGroup &Element::integration_points() const {
  return integration_points_;
}

}  // namespace ffea
