#include "../../inc/mesh/element.h"

#include "omp.h"

namespace ffea {

#pragma omp declare reduction (merge: MatrixEntries<double> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_out.end()))

Element::Element(GeometricEntity &entity,
                 const std::vector<DegreeOfFreedom *> &dofs,
                 const IntegrationPointsGroup &ips)
    : entity_(entity), dofs_(dofs), ips_(ips) {}

GeometricEntityType Element::geometric_entity_type() const {
  return entity_.type();
}

size_t Element::number_of_nodes() const { return entity_.number_of_nodes(); }

Coordinates &Element::node_coords(size_t node_idx) const {
  return entity_.node_coords(node_idx);
}

std::vector<size_t> Element::node_tags() const { return entity_.node_tags(); }

size_t Element::node_tag(size_t local_node_idx) const {
  return entity_.node_tag(local_node_idx);
}

size_t Element::number_of_dofs() const { return dofs_.size(); }

size_t Element::dofs_per_node() const {
  return number_of_dofs() / number_of_nodes();
}

std::vector<DegreeOfFreedom *> Element::dofs() const { return dofs_; }

std::vector<size_t> Element::dof_tags() const {
  std::vector<size_t> dof_tags;
  dof_tags.reserve(number_of_dofs());
  for (const auto &dof : dofs_) {
    dof_tags.push_back(dof->tag());
  }
  return dof_tags;
}

void Element::SetSparsity(MatrixEntries<double> &nonzero_entries) const {
  MatrixEntries<double> entries;
  // Number of entries of an upper-triangular element matrix
  const auto &number_of_entries = number_of_dofs() * (number_of_dofs() + 1) / 2;
  entries.reserve(number_of_entries);
  const auto &tags = dof_tags();

#pragma opm parallel for schedule(dynamic) reduction(merge : entries)
  for (size_t i_dof_idx = 0; i_dof_idx < number_of_dofs(); i_dof_idx++) {
    const auto &i_dof_tag = tags[i_dof_idx];
    for (size_t j_dof_idx = i_dof_idx; j_dof_idx < number_of_dofs();
         j_dof_idx++) {
      const auto &j_dof_tag = tags[j_dof_idx];
      if (i_dof_tag <= j_dof_tag) {
        entries.emplace_back(i_dof_tag, j_dof_tag);
      } else {
        entries.emplace_back(j_dof_tag, i_dof_tag);
      }
    }
  }
  nonzero_entries.insert(nonzero_entries.end(), entries.begin(), entries.end());
}

void Element::ProcessOverDomain(const ConstitutiveModel &constitutive_model,
                                Integrand integrand, ConditionFunction source,
                                CSRMatrix<double> &global_stiffness,
                                Vector<double> &global_rhs) const {
  ElementSystem system{};
  system.stiffness_matrix =
      Matrix<double>::Zero(number_of_dofs(), number_of_dofs());
  if (source) {
    system.rhs_vector = Vector<double>::Zero(number_of_dofs());
  }

  for (const auto &ip : ips_) {
    const auto &local_coords = ip.local_coords();
    const auto &N =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = MapLocalToGlobal(N);
    const auto &dN_local =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kFirst);
    const auto &jacobian = EvaluateJacobian(local_coords, dN_local);
    const auto &dN_global = jacobian.inverse() * dN_local;
    const auto &C = constitutive_model.Evaluate(global_coords);
    const auto &weight = ip.weight();
    const auto &differential = EvaluateDifferential(local_coords);

    (*system.stiffness_matrix).triangularView<Eigen::Upper>() +=
        integrand(dN_global, C) * weight * differential;

    if (source) {
      const auto &load_vector = source(global_coords);
      AddLoadContribution(load_vector, N, weight, differential, system);
    }
  }

  Scatter(system, global_stiffness, global_rhs);
}

void Element::ProcessOverBoundary(ConditionFunction load,
                                  ConditionFunction radiation,
                                  CSRMatrix<double> &global_stiffness,
                                  Vector<double> &global_rhs) const {
  ElementSystem system{};
  system.rhs_vector = Vector<double>::Zero(number_of_dofs());
  if (radiation) {
    system.stiffness_matrix =
        Matrix<double>::Zero(number_of_dofs(), number_of_dofs());
  }

  for (const auto &ip : ips_) {
    const auto &local_coords = ip.local_coords();
    const auto &N =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = MapLocalToGlobal(N);
    const auto &load_vector = load(global_coords);
    const auto &weight = ip.weight();
    const auto &differential = EvaluateDifferential(local_coords);

    AddLoadContribution(load_vector, N, weight, differential, system);

    if (radiation) {
      const auto radiation_value = radiation(global_coords)[0];
      AddRadiationContribution(radiation_value, N, weight, differential,
                               system);
    }
  }

  Scatter(system, global_stiffness, global_rhs);
}

Vector<double> Element::ExtractSolution() const {
  Vector<double> solution = Vector<double>::Zero(number_of_dofs());

  for (size_t dof_idx = 0; dof_idx < number_of_dofs(); dof_idx++) {
    solution(dof_idx) = dofs_[dof_idx]->value();
  }

  return solution;
}

void Element::AddNodalValues(
    ValuesProcessor values_processor,
    std::vector<ffea::NodalValuesGroup> &raw_values) const {
  const auto &nodal_local_coords = entity_.nodal_local_coords();
  const auto &solution = ExtractSolution();
  for (size_t node_idx = 0; node_idx < number_of_nodes(); node_idx++) {
    const auto &local_coords = nodal_local_coords[node_idx];
    const auto &N =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
    const auto &global_coords = MapLocalToGlobal(N);
    const auto &dN_local =
        EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kFirst);
    const auto &jacobian = EvaluateJacobian(local_coords, dN_local);
    const auto &dN_global = jacobian.inverse() * dN_local;
    const auto &values = values_processor(solution, global_coords, dN_global);
    std::vector<double> values_as_vector(values.data(),
                                         values.data() + values.size());
    const auto &node_tag = entity_.node_tag(node_idx);
    raw_values[node_tag].emplace_back(values.data(),
                                      values.data() + values.size());
  }
}

Matrix<double> Element::EvaluateShapeFunctions(const Coordinates &local_coords,
                                               DerivativeOrder order) const {
  return entity_.EvaluateShapeFunctions(local_coords, order);
}

Matrix<double> Element::EvaluateJacobian(const Coordinates &local_coords,
                                         const Matrix<double> &dN_local) const {
  return entity_.EvaluateJacobian(local_coords, dN_local);
}

Vector<double> Element::EvaluateNormalVector(
    const Coordinates &local_coords) const {
  return entity_.EvaluateNormalVector(local_coords);
}

double Element::EvaluateDifferential(const Coordinates &local_coords) const {
  return entity_.EvaluateDifferential(local_coords);
}

Coordinates Element::MapLocalToGlobal(const Coordinates &local_coords) const {
  return entity_.MapLocalToGlobal(local_coords);
}

Coordinates Element::MapLocalToGlobal(const Matrix<double> &N_at_point) const {
  return entity_.MapLocalToGlobal(N_at_point);
}

void Element::AddLoadContribution(const std::vector<double> &load_vector,
                                  const Matrix<double> &N, double weight,
                                  double differential,
                                  ElementSystem &system) const {
  for (auto node_idx = 0; node_idx < number_of_nodes(); node_idx++) {
    for (auto component_idx = 0; component_idx < dofs_per_node();
         component_idx++) {
      const auto &dof_idx = node_idx * dofs_per_node() + component_idx;
      (*system.rhs_vector)(dof_idx) +=
          N(0, node_idx) * load_vector[component_idx] * weight * differential;
    }
  }
}

void Element::AddRadiationContribution(double radiation,
                                       const Matrix<double> &N, double weight,
                                       double differential,
                                       ElementSystem &system) const {
  (*system.stiffness_matrix).triangularView<Eigen::Upper>() +=
      N.transpose() * radiation * N * weight * differential;
}

void Element::Scatter(const ElementSystem &element_system,
                      CSRMatrix<double> &global_stiffness,
                      Vector<double> &global_rhs) const {
  const auto &tags = dof_tags();

  for (size_t i_dof_idx = 0; i_dof_idx < number_of_dofs(); i_dof_idx++) {
    const auto &i_dof_tag = tags[i_dof_idx];
    for (size_t j_dof_idx = i_dof_idx; j_dof_idx < number_of_dofs();
         j_dof_idx++) {
      if (!element_system.stiffness_matrix) {
        continue;
      }

      const auto &value =
          (*element_system.stiffness_matrix)
              .selfadjointView<Eigen::Upper>()(i_dof_idx, j_dof_idx);
      const auto &j_dof_tag = tags[j_dof_idx];
      if (i_dof_tag <= j_dof_tag) {
        global_stiffness.coeffRef(i_dof_tag, j_dof_tag) += value;
      } else {
        global_stiffness.coeffRef(j_dof_tag, i_dof_tag) += value;
      }
    }

    if (element_system.rhs_vector) {
      global_rhs(i_dof_tag) += (*element_system.rhs_vector)(i_dof_idx);
    }
  }
}

}  // namespace ffea
