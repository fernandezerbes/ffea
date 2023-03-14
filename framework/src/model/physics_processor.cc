#include "../../inc/model/physics_processor.h"

namespace ffea {

PhysicsProcessor::PhysicsProcessor(const std::vector<Element> &elements) : elements_(elements) {}

void PhysicsProcessor::SetSparsity(MatrixEntries<double> &nonzero_entries) const {
  for (auto &element : elements_) {
    element.SetSparsity(nonzero_entries);
  }
}

void PhysicsProcessor::AddLoadContribution(const std::vector<double> &load_vector,
                                           const Matrix<double> &N, double weight,
                                           double differential, Vector<double> &element_rhs) const {
  const auto number_of_nodes = N.rows();
  const auto number_of_components = load_vector.size();
  for (auto node_idx = 0; node_idx < number_of_nodes; node_idx++) {
    for (auto component_idx = 0; component_idx < number_of_components; component_idx++) {
      const auto &dof_idx = node_idx * number_of_components + component_idx;
      element_rhs(dof_idx) += N(0, node_idx) * load_vector[component_idx] * weight * differential;
    }
  }
}

void PhysicsProcessor::Scatter(const std::vector<size_t> &dofs_tags,
                               const Matrix<double> &element_stiffness,
                               const Vector<double> &element_rhs,
                               CSRMatrix<double> &system_stiffness,
                               Vector<double> &system_rhs) const {
  for (size_t i_dof_idx = 0; i_dof_idx < dofs_tags.size(); i_dof_idx++) {
    const auto &i_dof_tag = dofs_tags[i_dof_idx];
    for (size_t j_dof_idx = i_dof_idx; j_dof_idx < dofs_tags.size(); j_dof_idx++) {
      const auto &value = element_stiffness.selfadjointView<Eigen::Upper>()(i_dof_idx, j_dof_idx);
      const auto &j_dof_tag = dofs_tags[j_dof_idx];
      if (i_dof_tag <= j_dof_tag) {
        system_stiffness.coeffRef(i_dof_tag, j_dof_tag) += value;
      } else {
        system_stiffness.coeffRef(j_dof_tag, i_dof_tag) += value;
      }
    }

    system_rhs(i_dof_tag) += element_rhs(i_dof_idx);
  }
}

void PhysicsProcessor::Scatter(const std::vector<size_t> &dofs_tags,
                               const Matrix<double> &element_stiffness,
                               CSRMatrix<double> &system_stiffness) const {
  for (size_t i_dof_idx = 0; i_dof_idx < dofs_tags.size(); i_dof_idx++) {
    const auto &i_dof_tag = dofs_tags[i_dof_idx];
    for (size_t j_dof_idx = i_dof_idx; j_dof_idx < dofs_tags.size(); j_dof_idx++) {
      const auto &value = element_stiffness.selfadjointView<Eigen::Upper>()(i_dof_idx, j_dof_idx);
      const auto &j_dof_tag = dofs_tags[j_dof_idx];
      if (i_dof_tag <= j_dof_tag) {
        system_stiffness.coeffRef(i_dof_tag, j_dof_tag) += value;
      } else {
        system_stiffness.coeffRef(j_dof_tag, i_dof_tag) += value;
      }
    }
  }
}

void PhysicsProcessor::Scatter(const std::vector<size_t> &dofs_tags,
                               const Vector<double> &element_rhs,
                               Vector<double> &system_rhs) const {
  for (size_t i_dof_idx = 0; i_dof_idx < dofs_tags.size(); i_dof_idx++) {
    const auto &i_dof_tag = dofs_tags[i_dof_idx];
    system_rhs(i_dof_tag) += element_rhs(i_dof_idx);
  }
}

}  // namespace ffea
