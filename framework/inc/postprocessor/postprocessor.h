#ifndef FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
#define FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_

#include <eigen3/Eigen/Dense>

#include "../mesh/element.h"
#include "../mesh/mesh.h"

namespace ffea {

class PostProcessor {
 public:
  virtual Eigen::VectorXd GetValuesAtNode(const Element& element,
                                size_t component_index) const = 0;
};

class DisplacementsPostProcessor : public PostProcessor {
 public:
  DisplacementsPostProcessor(const Mesh& mesh);
  virtual Eigen::VectorXd GetValuesAtNode(const Element& element,
                                size_t component_index) const override;

 private:
  const Mesh& mesh_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_POSTPROCESSOR_H_
