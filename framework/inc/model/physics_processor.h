#ifndef FFEA_FRAMEWORK_MODEL_PHYSICSPROCESSOR_H_
#define FFEA_FRAMEWORK_MODEL_PHYSICSPROCESSOR_H_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <optional>
#include <tuple>

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
  PhysicsProcessor(const std::vector<Element> &elements);
  void AddContribution(Eigen::MatrixXd &global_stiffness,
                       Eigen::VectorXd &global_rhs) const;

 protected:
  virtual ElementSystem ProcessElementSystem(const Element &element) const = 0;

 private:
  void Scatter(const Element &element, const ElementSystem &element_system,
               Eigen::MatrixXd &global_stiffness,
               Eigen::VectorXd &global_rhs) const;
  const std::vector<Element> &elements_;
};

class ElasticityDomainProcessor : public PhysicsProcessor {
 public:
  ElasticityDomainProcessor(const std::vector<Element> &elements,
                            const ConstitutiveModel &constitutive_model,
                            ConditionFunction source,
                            DifferentialOperator B_operator);
 
 protected:
  virtual ElementSystem ProcessElementSystem(
      const Element &element) const override;

 private:
  const ConstitutiveModel &constitutive_model_;
  ConditionFunction source_;
  DifferentialOperator B_operator_;
};

class ElasticityBoundaryProcessor : public PhysicsProcessor {
 public:
  ElasticityBoundaryProcessor(const std::vector<Element> &elements,
                              size_t dimensions, ConditionFunction load);

 protected:
  virtual ElementSystem ProcessElementSystem(
      const Element &element) const override;

 private:
  size_t dimensions_;
  ConditionFunction load_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_PHYSICSPROCESSOR_H_
