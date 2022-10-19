#ifndef FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
#define FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include "../geometry/coordinates.h"
#include "../mesh/mesh.h"
#include "../model/constitutive_model.h"
#include "../model/operator.h"
#include "../model/types.h"

namespace ffea {

class PostProcessor {
 public:
  PostProcessor(std::string variable_name, size_t values_per_node,
                const Mesh &mesh);
  virtual ~PostProcessor() = default;

  virtual std::vector<double> Process(const std::string &group_name) const = 0;
  std::string variable_name() const;
  size_t values_per_node() const;

 protected:
  const Mesh &mesh_;

 private:
  std::string variable_name_;
  size_t values_per_node_;
};

class PrimaryVariablePostProcessor : public PostProcessor {
 public:
  PrimaryVariablePostProcessor(std::string variable_name,
                               size_t values_per_node, const Mesh &mesh);

  virtual std::vector<double> Process(
      const std::string &group_name) const override;
};

class DerivedVariableProcessor : public PostProcessor {
 public:
  DerivedVariableProcessor(std::string variable_name, size_t values_per_node,
                           const Mesh &mesh, ValuesProcessor processor);

  virtual std::vector<double> Process(
      const std::string &group_name) const override;

 private:
  std::vector<ffea::NodalValuesGroup> ExtractValuesOfAllElementsPerNode(
      const std::string &group_name) const;
  std::vector<double> AverageNodalValues(
      const std::vector<ffea::NodalValuesGroup> &raw_values) const;

  ValuesProcessor processor_;
};

namespace utilities {

PrimaryVariablePostProcessor MakeDisplacementProcessor2D(const Mesh &mesh);

PrimaryVariablePostProcessor MakeDisplacementProcessor3D(const Mesh &mesh);

PrimaryVariablePostProcessor MakeTemperatureProcessor(const Mesh &mesh);

DerivedVariableProcessor MakeElasticStrainProcessor(
    size_t values_per_node, const Mesh &mesh, DifferentialOperator B_operator);

DerivedVariableProcessor MakeElasticStrainProcessor2D(const Mesh &mesh);

DerivedVariableProcessor MakeElasticStrainProcessor3D(const Mesh &mesh);

DerivedVariableProcessor MakeElasticStressProcessor(
    size_t values_per_node, const Mesh &mesh,
    const ConstitutiveModel &constitutive_model,
    DifferentialOperator B_operator);

DerivedVariableProcessor MakeElasticStressProcessor2D(
    const Mesh &mesh, const ConstitutiveModel &constitutive_model);

DerivedVariableProcessor MakeElasticStressProcessor3D(
    const Mesh &mesh, const ConstitutiveModel &constitutive_model);

}  // namespace utilities

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
