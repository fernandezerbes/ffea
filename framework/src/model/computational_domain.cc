#include "../../inc/model/computational_domain.h"

namespace ffea {

ComputationalDomain::ComputationalDomain(
    const std::vector<Element>& elements,
    const ConstitutiveModel& constitutive_model, Integrand integrand,
    ConditionFunction source)
    : elements_(elements),
      constitutive_model_(constitutive_model),
      source_(source),
      integrand_(integrand) {}

void ComputationalDomain::AddContribution(Eigen::MatrixXd& global_stiffness,
                                          Eigen::VectorXd& global_rhs) const {
  for (auto& element : elements_) {
    element.ProcessOverDomain(constitutive_model_, integrand_, source_,
                              global_stiffness, global_rhs);
  }
}

}  // namespace ffea
