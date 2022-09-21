#include "../../inc/model/computational_domain.h"

namespace ffea {

ComputationalDomain::ComputationalDomain(
    const std::vector<Element>& domain_elements, const Integrand& integrand)
    : domain_elements_(domain_elements), integrand_(integrand) {}

void ComputationalDomain::AddContribution(Eigen::MatrixXd& global_stiffness,
                                          Eigen::VectorXd& global_rhs) const {
  for (auto& element : domain_elements_) {
    const auto& element_K = element.ComputeStiffness(integrand_);
    // const auto& element_rhs = element.ComputeRhs(source_);
    const auto& dofs_map = element.GetLocalToGlobalDofIndicesMap();

    // Scatter coefficients
    for (size_t node_index = 0; node_index < element.GetNumberOfNodes();
         node_index++) {
      size_t local_i_index = 0;
      for (const auto& global_i_index : dofs_map) {
        size_t local_j_index = 0;
        for (const auto& global_j_index : dofs_map) {
          global_stiffness(global_i_index, global_j_index) +=
              element_K(local_i_index, local_j_index);
          local_j_index++;
        }
        // global_rhs(global_i_index) += element_rhs(local_i_index);
        local_i_index++;
      }
    }
  }
}

}  // namespace ffea
