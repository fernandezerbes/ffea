#ifndef FFEA_FRAMEWORK_MODEL_INTEGRAND_H_
#define FFEA_FRAMEWORK_MODEL_INTEGRAND_H_

#include <eigen3/Eigen/Dense>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "./constitutive_model.h"
#include "./types.h"

namespace ffea {

class Integrand {
 public:
  explicit Integrand(size_t physical_dimension);

  virtual const Eigen::MatrixXd Compute(
      const GeometricEntity &geometric_entity,
      const Coordinates &local_coordinates) const = 0;

 protected:
  size_t physical_dimension_;
};

class ElasticityBaseIntegrand : public Integrand {
 public:
  ElasticityBaseIntegrand(size_t physical_dimension,
                          const ConstitutiveModel &constitutive_model,
                          ConditionFunction source);

  virtual const Eigen::MatrixXd Compute(
      const GeometricEntity &geometric_entity,
      const Coordinates &local_coordinates) const override;

 protected:
  const ConstitutiveModel &constitutive_model_;
  ConditionFunction source_;

 private:
  virtual const Eigen::MatrixXd GetStrainDisplacementOperator(
      const Eigen::MatrixXd &shape_function_derivatives) const = 0;
};

class Elasticity2DIntegrand : public ElasticityBaseIntegrand {
 public:
  Elasticity2DIntegrand(const ConstitutiveModel &constitutive_model,
                        ConditionFunction source);

 private:
  virtual const Eigen::MatrixXd GetStrainDisplacementOperator(
      const Eigen::MatrixXd &shape_function_derivatives) const override;
};

class Elasticity3DIntegrand : public ElasticityBaseIntegrand {
 public:
  Elasticity3DIntegrand(const ConstitutiveModel &constitutive_model,
                        ConditionFunction source);

 private:
  virtual const Eigen::MatrixXd GetStrainDisplacementOperator(
      const Eigen::MatrixXd &shape_function_derivatives) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MODEL_INTEGRAND_H_