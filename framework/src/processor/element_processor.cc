#include "../../inc/processor/element_processor.h"
namespace ffea {

ElementProcessor::ElementProcessor(Eigen::MatrixXd constitutive_model)
    : constitutive_model_(constitutive_model) {}

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
ElementProcessor::ProcessBodyElement(const Element& element,
                                     LoadFunction body_load_function) const {
  size_t number_of_dofs = element.GetNumberOfDofs();
  Eigen::MatrixXd element_stiffness(number_of_dofs, number_of_dofs);
  Eigen::VectorXd element_rhs(number_of_dofs);
  for (const auto& integration_point : *element.integration_points()) {
    const auto& local_coordinates = integration_point.local_coordinates();
    const auto& jacobian = element.EvaluateJacobian(local_coordinates);
    AddContributionToStiffness(element, integration_point, jacobian,
                               element_stiffness);
    AddContributionToRhs(element, integration_point, jacobian,
                         body_load_function, element_rhs);
  }
  return {element_stiffness, element_rhs};
}

Eigen::VectorXd ElementProcessor::ProcessBoundaryElement(
    const Element& element, LoadFunction boundary_load) const {
  size_t number_of_dofs = element.GetNumberOfDofs();
  Eigen::VectorXd element_rhs(number_of_dofs);
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
  Eigen::MatrixXd B(3, element.GetNumberOfDofs());
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
  stiffness += B.transpose() * constitutive_model_ * B *
               jacobian.determinant() * integration_point.weight();
}

void ElementProcessor::AddContributionToRhs(
    const Element& element, const IntegrationPoint& integration_point,
    const Eigen::MatrixXd& jacobian, LoadFunction load_function,
    Eigen::VectorXd& rhs) const {
  const auto& local_coordinates = integration_point.local_coordinates();
  const auto& shape_functions = element.EvaluateShapeFunctions(
      local_coordinates, ffea::DerivativeOrder::kZeroth);
  const auto& global_coordinates = element.MapLocalToGlobal(local_coordinates);
  const auto& body_load = load_function(global_coordinates);
  for (size_t dimension_index = 0; dimension_index < element.dimension();
       dimension_index++) {
    rhs += shape_functions.transpose() * body_load[dimension_index] *
           jacobian.determinant() * integration_point.weight();
  }
}

}  // namespace ffea
