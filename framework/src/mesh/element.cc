#include "../../inc/mesh/element.h"

namespace ffea {

Element::Element(size_t dimension, const std::vector<Node *> &nodes,
                 const ShapeFunctions &shape_functions,
                 IntegrationPointsGroupPtr integration_points)
    : dimension_(dimension),
      nodes_(nodes),
      shape_functions_(shape_functions),
      integration_points_(integration_points) {}

Element::~Element() {}

size_t Element::dimension() const { return dimension_; }

std::vector<Node *> &Element::nodes() { return nodes_; }

IntegrationPointsGroupPtr Element::integration_points() const {
  return integration_points_;
}

std::vector<size_t> Element::GetLocalToGlobalDofIndicesMap() const {
  std::vector<size_t> indices_map;
  indices_map.reserve(GetNumberOfDofs());
  for (const auto &node : nodes_) {
    size_t dofs_per_node = GetNumberOfDofsPerNode();
    for (size_t dof_index = 0; dof_index < dofs_per_node; dof_index++) {
      // TODO This will be problematic with meshes that have nodes with
      // different number of dofs_per_node
      indices_map.push_back(dofs_per_node * node->id() + dof_index);
    }
  }
  return indices_map;
}

size_t Element::GetNumberOfNodes() const { return nodes_.size(); }

size_t Element::GetNumberOfDofsPerNode() const {
  return nodes_[0]->number_of_dofs();
}

size_t Element::GetNumberOfDofs() const {
  return GetNumberOfNodes() * GetNumberOfDofsPerNode();
}

Eigen::MatrixXd Element::EvaluateShapeFunctions(
    const Coordinates &local_coordinates,
    DerivativeOrder derivative_order) const {
  return shape_functions_.Evaluate(local_coordinates.get(), derivative_order);
}

Eigen::MatrixXd Element::GetNodesCoordinatesValues() const {
  Eigen::MatrixXd coordinates_values(GetNumberOfNodes(), dimension_);
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    const auto &coordinates = node->coordinates();
    for (size_t dimension_index = 0; dimension_index < dimension_;
         dimension_index++) {
      coordinates_values(node_index, dimension_index) =
          coordinates.get(dimension_index);
    }
  }
  return coordinates_values;
}

Eigen::MatrixXd Element::EvaluateJacobian(
    const Coordinates &local_coordinates) const {
  Eigen::MatrixXd jacobian(dimension_, dimension_);
  const auto &shape_functions_derivatives =
      EvaluateShapeFunctions(local_coordinates, DerivativeOrder::kFirst);
  const auto &nodes_coordinates_values = GetNodesCoordinatesValues();
  return shape_functions_derivatives * nodes_coordinates_values;
}

Coordinates Element::MapLocalToGlobal(
    const Coordinates &local_coordinates,
    const Eigen::MatrixXd &shape_functions) const {
  // TODO Maybe this should be always 3: std::array<double, 3>
  std::vector<double> xyz(dimension_, 0.0);
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    const auto &coordinates = node->coordinates();
    // TODO Dimension index will be always less than 3
    // Maybe we can always use the three dimensions
    for (size_t dimension_index = 0; dimension_index < dimension_;
         dimension_index++) {
      xyz[dimension_index] +=
          shape_functions(0, node_index) * coordinates.get(dimension_index);
    }
  }
  // TODO this can be avoided if we create the coordinates before
  return Coordinates(xyz);
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
    const auto &jacobian = EvaluateJacobian(local_coordinates);
    const auto &dN_local = EvaluateShapeFunctions(
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
    const auto &shape_functions = EvaluateShapeFunctions(
        local_coordinates, ffea::DerivativeOrder::kZeroth);
    const auto &global_coordinates =
        MapLocalToGlobal(local_coordinates, shape_functions);
    const auto &body_load = load(global_coordinates);
    const auto &jacobian = EvaluateJacobian(local_coordinates);

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

Coordinates &Element::GetCoordinatesOfNode(size_t node_index) const {
  return nodes_[node_index]->coordinates();
}

void Element::SetSolutionOnDofs(const Eigen::VectorXd &solution) {
  for (auto &node : nodes_) {
    node->SetSolutionOnDofs(solution);
  }
}

Eigen::VectorXd Element::GetSolutionFromDofs() const {
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(GetNumberOfDofs());

  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    for (size_t component_index = 0; component_index < GetNumberOfDofsPerNode();
         component_index++) {
      size_t index = node_index * GetNumberOfDofsPerNode() + component_index;
      solution(index) = node->GetSolutionOfDof(component_index);
    }
  }

  return solution;
}

Eigen::VectorXd Element::GetSolutionFromDofs(size_t component_index) const {
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(GetNumberOfNodes());

  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    size_t index = node_index * GetNumberOfDofsPerNode() + component_index;
    solution(node_index) = node->GetSolutionOfDof(component_index);
  }

  return solution;
}

}  // namespace ffea
