#include "../../inc/mesh/element.h"

namespace ffea {

Element::Element(GeometricEntity &geometric_entity,
                 const std::vector<DegreeOfFreedom *> &dofs,
                 const Quadrature &quadrature)
    : geometric_entity_(geometric_entity),
      dofs_(dofs),
      quadrature_(quadrature) {}

Element::~Element() {}

IntegrationPointsGroupPtr Element::integration_points() const {
  return quadrature_.GetIntegrationPoints();
}

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

Eigen::MatrixXd Element::ComputeStiffness(
    const Eigen::MatrixXd &constitutive_model,
    const DifferentialOperator &differential_operator) const {
  size_t number_of_dofs = GetNumberOfDofs();
  Eigen::MatrixXd stiffness =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);

  for (const auto &integration_point : *integration_points()) {
    const auto &local_coordinates = integration_point.local_coordinates();
    // Jacobian can be evaluated outside, to be shared with ComputeRhs
    const auto &jacobian =
        geometric_entity_.EvaluateJacobian(local_coordinates);
    const auto &dN_local = geometric_entity_.EvaluateShapeFunctions(
        local_coordinates, ffea::DerivativeOrder::kFirst);
    const auto &dN_global = jacobian.inverse() * dN_local;
    Eigen::MatrixXd operator_matrix = differential_operator.Compute(dN_global);

    stiffness += operator_matrix.transpose() * constitutive_model *
                 operator_matrix * jacobian.determinant() *
                 integration_point.weight();
  }

  return stiffness;
}

Eigen::VectorXd Element::ComputeRhs(ConditionFunction load) const {
  size_t number_of_dofs = GetNumberOfDofs();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(number_of_dofs);

  for (const auto &integration_point : *integration_points()) {
    const auto &local_coordinates = integration_point.local_coordinates();
    const auto &shape_functions = geometric_entity_.EvaluateShapeFunctions(
        local_coordinates, ffea::DerivativeOrder::kZeroth);
    const auto &global_coordinates =
        geometric_entity_.MapLocalToGlobal(local_coordinates, shape_functions);
    const auto &body_load = load(global_coordinates);
    const auto &jacobian =
        geometric_entity_.EvaluateJacobian(local_coordinates);

    auto number_of_load_components = GetNumberOfDofsPerNode();
    for (size_t load_component_index = 0;
         load_component_index < number_of_load_components;
         load_component_index++) {
      const auto &load_component_contribution =
          shape_functions * body_load[load_component_index] *
          jacobian.determinant() * integration_point.weight();
      for (size_t node_index = 0; node_index < GetNumberOfNodes();
           node_index++) {
        auto dof_index =
            node_index * number_of_load_components + load_component_index;
        rhs(dof_index) += load_component_contribution(0, node_index);
      }
    }
  }

  return rhs;
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

}  // namespace ffea
