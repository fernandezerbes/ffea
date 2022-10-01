#ifndef FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
#define FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_

#include <eigen3/Eigen/Dense>
#include <string>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./physics_processor.h"

namespace ffea {

class ComputationalDomain {
 public:
  ComputationalDomain(const std::vector<Element> &domain_elements,
                      std::unique_ptr<PhysicsProcessor> processor);

  void AddContribution(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const;

 private:
  const std::vector<Element> &domain_elements_;
  std::unique_ptr<PhysicsProcessor> processor_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_COMPUTATIONALDOMAIN_H_
