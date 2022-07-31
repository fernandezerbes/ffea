#include "../../inc/mesh/element.h"

namespace ffea {

Element::Element(size_t dimension, const std::vector<Node *> &nodes,
                 std::shared_ptr<ShapeFunctions> shape_functions,
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

std::vector<int> Element::GetLocalToGlobalDofIndicesMap() const {
  std::vector<int> indices_map;
  indices_map.reserve(GetNumberOfDofs());
  for (const auto &node : nodes_) {
    size_t dofs_per_node = GetNumberOfDofsPerNode();
    for (size_t dof_index = 0; dof_index < dofs_per_node; dof_index++) {
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
  return shape_functions_->Evaluate(local_coordinates.get(), derivative_order);
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
    const Coordinates &local_coordinates) const {
  const auto &shape_functions =
      EvaluateShapeFunctions(local_coordinates, DerivativeOrder::kZeroth);

  std::vector<double> xyz(dimension_, 0.0);
  for (size_t node_index = 0; node_index < GetNumberOfNodes(); node_index++) {
    const auto &node = nodes_[node_index];
    const auto &coordinates = node->coordinates();
    for (size_t dimension_index = 0; dimension_index < dimension_;
         dimension_index++) {
      xyz[dimension_index] +=
          shape_functions(0, node_index) * coordinates.get(dimension_index);
    }
  }

  return Coordinates(xyz);
}

ElementFactory::ElementFactory(size_t dimension,
                               std::shared_ptr<ShapeFunctions> shape_functions,
                               std::shared_ptr<QuadratureRule> integration_rule)
    : dimension_(dimension),
      shape_functions_(shape_functions),
      integration_points_(nullptr) {
  integration_points_ = integration_rule->GetIntegrationPoints();
}

Element ElementFactory::CreateElement(const std::vector<Node *> &nodes) const {
  return ffea::Element(dimension_, nodes, shape_functions_,
                       integration_points_);
}

}  // namespace ffea
