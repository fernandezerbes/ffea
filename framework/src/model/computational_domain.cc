#include "../../inc/model/computational_domain.h"

namespace ffea {

ComputationalDomain::ComputationalDomain(
    const std::vector<Element>& elements,
    const ConstitutiveModel& constitutive_model, ConditionFunction source,
    const PhysicsProcessor& processor)
    : elements_(elements),
      constitutive_model_(constitutive_model),
      source_(source),
      processor_(processor) {}

void ComputationalDomain::AddContribution(Eigen::MatrixXd& global_stiffness,
                                          Eigen::VectorXd& global_rhs) const {
  processor_.AddDomainContribution(elements_, constitutive_model_, source_,
                                   global_stiffness, global_rhs);
}

}  // namespace ffea
