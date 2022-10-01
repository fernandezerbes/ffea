#include "../../inc/model/computational_domain.h"

namespace ffea {

ComputationalDomain::ComputationalDomain(
    std::unique_ptr<PhysicsProcessor> processor)
    : processor_(std::move(processor)) {}

void ComputationalDomain::AddContribution(Eigen::MatrixXd& global_stiffness,
                                          Eigen::VectorXd& global_rhs) const {
  processor_->AddContribution(global_stiffness, global_rhs);
}

}  // namespace ffea
