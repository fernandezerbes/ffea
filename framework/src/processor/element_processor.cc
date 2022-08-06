#include "../../inc/processor/element_processor.h"
namespace ffea {

ElementProcessor::ElementProcessor(Eigen::MatrixXd constitutive_model)
    : constitutive_model_(constitutive_model) {}

ElementProcessor::~ElementProcessor() {}

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
ElementProcessor::ProcessBodyElement(const Element& element,
                                     LoadFunction body_load_function) const {
  size_t number_of_dofs = element.GetNumberOfDofs();
  Eigen::MatrixXd element_stiffness = Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  Eigen::VectorXd element_rhs = Eigen::VectorXd::Zero(number_of_dofs);
  for (const auto& integration_point : *element.integration_points()) {
    const auto& local_coordinates = integration_point.local_coordinates();
    const auto& jacobian = element.EvaluateJacobian(local_coordinates);
    AddContributionToStiffness(element, integration_point, jacobian,
                               element_stiffness);
    if (body_load_function) {
      AddContributionToRhs(element, integration_point, jacobian,
                           body_load_function, element_rhs);
    }
  }
  return {element_stiffness, element_rhs};
}

Eigen::VectorXd ElementProcessor::ProcessBoundaryElement(
    const Element& element, LoadFunction boundary_load) const {
  size_t number_of_dofs = element.GetNumberOfDofs();
  Eigen::VectorXd element_rhs = Eigen::VectorXd::Zero(number_of_dofs);
  for (const auto& integration_point : *element.integration_points()) {
    const auto& local_coordinates = integration_point.local_coordinates();
    const auto& jacobian = element.EvaluateJacobian(local_coordinates);
    AddContributionToRhs(element, integration_point, jacobian, boundary_load,
                         element_rhs);
  }
  return element_rhs;
}

void ElementProcessor::AddContributionToStiffness(
    const Element& element, const IntegrationPoint& integration_point,
    const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& stiffness) const {
  const auto& local_coordinates = integration_point.local_coordinates();
  const auto& dN_local = element.EvaluateShapeFunctions(
      local_coordinates, ffea::DerivativeOrder::kFirst);
  const auto& dN_global = jacobian.inverse() * dN_local;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, element.GetNumberOfDofs());
  size_t number_of_nodes = element.GetNumberOfNodes();
  size_t number_of_dofs_per_node = element.GetNumberOfDofsPerNode();
  for (size_t node_index = 0; node_index < number_of_nodes; node_index++) {
    auto first_dof_index = number_of_dofs_per_node * node_index;
    auto second_dof_index = first_dof_index + 1;
    B(0, first_dof_index) = dN_global(0, node_index);
    B(1, second_dof_index) = dN_global(1, node_index);
    B(2, first_dof_index) = dN_global(1, node_index);
    B(2, second_dof_index) = dN_global(0, node_index);
  }
  const auto& contribution = B.transpose() * constitutive_model_ * B *
               jacobian.determinant() * integration_point.weight();
  stiffness += contribution;
}

void ElementProcessor::AddContributionToRhs(
    const Element& element, const IntegrationPoint& integration_point,
    const Eigen::MatrixXd& jacobian, LoadFunction load_function,
    Eigen::VectorXd& rhs) const {
  size_t spatial_dimensions = 2;  // TODO Change this
  const auto& local_coordinates = integration_point.local_coordinates();
  const auto& shape_functions = element.EvaluateShapeFunctions(
      local_coordinates, ffea::DerivativeOrder::kZeroth);
  const auto& global_coordinates = element.MapLocalToGlobal(local_coordinates);
  const auto& body_load = load_function(global_coordinates);
  for (size_t dimension_index = 0; dimension_index < spatial_dimensions;
       dimension_index++) {
    const auto& load_components = shape_functions * body_load[dimension_index] *
                                  jacobian.determinant() *
                                  integration_point.weight();
    for (size_t component_index = 0; component_index < spatial_dimensions;
         component_index++) {
      rhs(dimension_index + component_index * spatial_dimensions) +=
          load_components(0, component_index);
    }
  }
}

}  // namespace ffea
