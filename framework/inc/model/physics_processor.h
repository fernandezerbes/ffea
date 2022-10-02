#ifndef FFEA_FRAMEWORK_MODEL_PHYSICSPROCESSOR_H_
#define FFEA_FRAMEWORK_MODEL_PHYSICSPROCESSOR_H_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <optional>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../mesh/element.h"
#include "./constitutive_model.h"
#include "./operator.h"
#include "./types.h"

namespace ffea {

struct ElementSystem {
  std::optional<Eigen::MatrixXd> mass_matrix;
  std::optional<Eigen::MatrixXd> damping_matrix;
  std::optional<Eigen::MatrixXd> stiffness_matrix;
  std::optional<Eigen::VectorXd> rhs_vector;
};

class PhysicsProcessor {
 public:
  void AddDomainContribution(const std::vector<Element> &elements,
                             const ConstitutiveModel &constitutive_model,
                             ConditionFunction source,
                             Eigen::MatrixXd &global_stiffness,
                             Eigen::VectorXd &global_rhs) const;
  void AddBoundaryContribution(const std::vector<Element> &elements,
                               ConditionFunction load,
                               Eigen::MatrixXd &global_stiffness,
                               Eigen::VectorXd &global_rhs) const;

 protected:
  virtual ElementSystem ProcessDomainElementSystem(
      const Element &element, const ConstitutiveModel &constitutive_model,
      ConditionFunction source) const = 0;
  virtual ElementSystem ProcessBoundaryElementSystem(
      const Element &element, ConditionFunction load) const = 0;
  void AddLoadContributionToElementSystem(
      size_t number_of_nodes, size_t number_of_components,
      const Eigen::MatrixXd &N, const std::vector<double> &load_vector,
      double weight, double differential, ElementSystem &system) const;

 private:
  void Scatter(const Element &element, const ElementSystem &element_system,
               Eigen::MatrixXd &global_stiffness,
               Eigen::VectorXd &global_rhs) const;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_PHYSICSPROCESSOR_H_
