#ifndef FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
#define FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_

#include <string>
#include <vector>

#include "../alias.h"
#include "../geometry/coordinates.h"
#include "../mesh/mesh.h"
#include "../model/constitutive_model.h"
#include "../model/operator.h"

namespace ffea {

class PostProcessor {
 public:
  PostProcessor(std::string variable_name, size_t values_per_node, const Mesh &mesh);
  virtual ~PostProcessor() = default;

  std::string variable_name() const;
  size_t values_per_node() const;

  virtual NodalValues Process(const std::string &group_name) const = 0;

 protected:
  const Mesh &mesh_;

 private:
  std::string variable_name_;
  size_t values_per_node_;
};

class PrimaryVariablePostProcessor : public PostProcessor {
 public:
  PrimaryVariablePostProcessor(std::string variable_name, size_t values_per_node, const Mesh &mesh);

  NodalValues Process(const std::string &group_name) const override;
};

class DerivedVariableProcessor : public PostProcessor {
 public:
  DerivedVariableProcessor(std::string variable_name, size_t values_per_node, const Mesh &mesh,
                           ValuesProcessor processor);

  NodalValues Process(const std::string &group_name) const override;

 private:
  std::vector<ffea::NodalValuesGroup> ExtractValuesOfAllElementsPerNode(
      const std::string &group_name) const;
  std::vector<double> AverageNodalValues(
      const std::vector<ffea::NodalValuesGroup> &raw_values) const;

  ValuesProcessor processor_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
