#include "../../inc/mesh/element.h"

namespace ffea {

Element::Element(GeometricEntity &geometric_entity,
                 const std::vector<DegreeOfFreedom *> &dofs,
                 const IntegrationPointsGroup &integration_points)
    : geometric_entity_(geometric_entity),
      dofs_(dofs),
      integration_points_(integration_points) {}

GeometricEntityType Element::GetGeometricEntityType() const {
  return geometric_entity_.type();
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

Coordinates &Element::GetCoordinatesOfNode(size_t node_idx) const {
  return geometric_entity_.GetCoordinatesOfNode(node_idx);
}

Eigen::VectorXd Element::GetSolutionFromDofs(size_t component_idx) const {
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(GetNumberOfNodes());

  size_t dof_idx = component_idx;
  for (size_t node_idx = 0; node_idx < GetNumberOfNodes(); node_idx++) {
    const auto &dof = dofs_[dof_idx];
    solution(node_idx) = dof->value();
    dof_idx += 2;
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

void Element::ProcessOverDomain(const ConstitutiveModel &constitutive_model,
                                Integrand integrand, ConditionFunction source,
                                Eigen::MatrixXd &global_stiffness,
                                Eigen::VectorXd &global_rhs) const {
  ElementSystem system{};
  const auto &number_of_dofs = GetNumberOfDofs();
  system.stiffness_matrix =
      Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  if (source) {
    system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);
  }

  for (const auto &integration_point : integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &N =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = MapLocalToGlobal(N);
    const auto &dN_dLocal =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kFirst);
    const auto &jacobian = EvaluateJacobian(local_coords, dN_dLocal);
    const auto &dN_dGlobal = jacobian.inverse() * dN_dLocal;
    const auto &C = constitutive_model.Evaluate(global_coords);
    const auto &weight = integration_point.weight();
    const auto &differential = EvaluateDifferential(local_coords);

    *(system.stiffness_matrix) +=
        integrand(dN_dGlobal, C) * weight * differential;

    if (source) {
      const auto &load_vector = source(global_coords);
      AddLoadContribution(load_vector, N, weight, differential, system);
    }
  }

  Scatter(system, global_stiffness, global_rhs);
}

void Element::ProcessOverBoundary(ConditionFunction load,
                                  ConditionFunction radiation,
                                  Eigen::MatrixXd &global_stiffness,
                                  Eigen::VectorXd &global_rhs) const {
  ElementSystem system{};
  const auto &number_of_dofs = GetNumberOfDofs();
  system.rhs_vector = Eigen::VectorXd::Zero(number_of_dofs);
  if (radiation) {
    system.stiffness_matrix =
        Eigen::MatrixXd::Zero(number_of_dofs, number_of_dofs);
  }

  for (const auto &integration_point : integration_points()) {
    const auto &local_coords = integration_point.local_coords();
    const auto &N =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = MapLocalToGlobal(N);
    const auto &load_vector = load(global_coords);
    const auto &weight = integration_point.weight();
    const auto &differential = EvaluateDifferential(local_coords);

    AddLoadContribution(load_vector, N, weight, differential, system);

    if (radiation) {
      const auto &radiation_value = radiation(global_coords)[0];
      AddRadiationContribution(radiation_value, N, weight, differential,
                               system);
    }
  }

  Scatter(system, global_stiffness, global_rhs);
}

void Element::AddLoadContribution(const std::vector<double> &load_vector,
                                  const Eigen::MatrixXd &N, double weight,
                                  double differential,
                                  ElementSystem &system) const {
  for (auto node_idx = 0; node_idx < GetNumberOfNodes(); node_idx++) {
    for (auto component_idx = 0; component_idx < GetNumberOfDofsPerNode();
         component_idx++) {
      const auto &dof_idx = node_idx * GetNumberOfDofsPerNode() + component_idx;
      (*system.rhs_vector)(dof_idx) +=
          N(0, node_idx) * load_vector[component_idx] * weight * differential;
    }
  }
}

void Element::AddRadiationContribution(double radiation,
                                       const Eigen::MatrixXd &N, double weight,
                                       double differential,
                                       ElementSystem &system) const {
  (*system.stiffness_matrix) +=
      N.transpose() * radiation * N * weight * differential;
}

void Element::Scatter(const ElementSystem &element_system,
                      Eigen::MatrixXd &global_stiffness,
                      Eigen::VectorXd &global_rhs) const {
  const auto &dofs_map = GetLocalToGlobalDofIndicesMap();
  for (size_t node_idx = 0; node_idx < GetNumberOfNodes(); node_idx++) {
    size_t local_i_idx = 0;
    for (const auto &global_i_idx : dofs_map) {
      size_t local_j_idx = 0;
      for (const auto &global_j_idx : dofs_map) {
        if (element_system.stiffness_matrix) {
          global_stiffness(global_i_idx, global_j_idx) +=
              (*element_system.stiffness_matrix)(local_i_idx, local_j_idx);
        }
        local_j_idx++;
      }
      if (element_system.rhs_vector) {
        global_rhs(global_i_idx) += (*element_system.rhs_vector)(local_i_idx);
      }
      local_i_idx++;
    }
  }
}

}  // namespace ffea
