#ifndef FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
#define FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include "../geometry/coordinates.h"
#include "../mesh/mesh.h"
#include "../model/constitutive_model.h"
#include "../model/operator.h"
#include "../model/alias.h"

namespace ffea {

class PostProcessor {
 public:
  PostProcessor(std::string variable_name, size_t values_per_node,
                const Mesh &mesh);
  virtual ~PostProcessor() = default;

  std::string variable_name() const;
  size_t values_per_node() const;
  
  virtual std::vector<double> Process(const std::string &group_name) const = 0;

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

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
