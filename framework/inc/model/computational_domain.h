#ifndef FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
#define FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_

#include <eigen3/Eigen/Dense>
#include <string>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "../model/operator.h"
#include "./constitutive_model.h"

namespace ffea {

class ComputationalDomain {
 public:
  ComputationalDomain(const std::vector<Element> &domain_elements,
                      const ConstitutiveModel &constitutive_model,
                      const DifferentialOperator &differential_operator,
                      ConditionFunction source);

  void AddContribution(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const;

 private:
  const std::vector<Element> &domain_elements_;
  const ConstitutiveModel &constitutive_model_;
  const DifferentialOperator &differential_operator_;
  ConditionFunction source_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
