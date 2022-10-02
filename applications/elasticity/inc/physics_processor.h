#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_PHYSICSPROCESSOR_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_PHYSICSPROCESSOR_H_

#include "../../../framework/inc/model/physics_processor.h"

namespace ffea {

class ElasticityProcessor : public PhysicsProcessor {
 public:
  ElasticityProcessor(DifferentialOperator B_operator);

 protected:
  virtual ElementSystem ProcessDomainElementSystem(
      const Element &element, const ConstitutiveModel &constitutive_model,
      ConditionFunction source) const override;
  virtual ElementSystem ProcessBoundaryElementSystem(
      const Element &element, ConditionFunction load) const override;

 private:
  DifferentialOperator B_operator_;
};

}  // namespace ffea

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_PHYSICSPROCESSOR_H_
