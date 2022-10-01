#include "../../inc/model/computational_domain.h"

namespace ffea {

ComputationalDomain::ComputationalDomain(
    const std::vector<Element>& domain_elements,
    std::unique_ptr<PhysicsProcessor> processor)
    : domain_elements_(domain_elements), processor_(std::move(processor)) {}

void ComputationalDomain::AddContribution(Eigen::MatrixXd& global_stiffness,
                                          Eigen::VectorXd& global_rhs) const {
  for (auto& element : domain_elements_) {
    const auto& system = processor_->ProcessElementSystem(element);
    const auto& dofs_map = element.GetLocalToGlobalDofIndicesMap();

    // Scatter coefficients
    for (size_t node_index = 0; node_index < element.GetNumberOfNodes();
         node_index++) {
      size_t local_i_index = 0;
      for (const auto& global_i_index : dofs_map) {
        size_t local_j_index = 0;
        for (const auto& global_j_index : dofs_map) {
          if (system.stiffness_matrix) {
            global_stiffness(global_i_index, global_j_index) +=
                (*system.stiffness_matrix)(local_i_index, local_j_index);
          }
          local_j_index++;
        }
        if (system.rhs_vector) {
          global_rhs(global_i_index) += (*system.rhs_vector)(local_i_index);
        }
        local_i_index++;
      }
    }
  }
}

}  // namespace ffea
