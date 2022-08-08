#include "../../inc/postprocessor/postprocessor.h"

namespace ffea {

DisplacementsPostProcessor::DisplacementsPostProcessor(const Mesh& mesh)
    : mesh_(mesh) {}

Eigen::VectorXd DisplacementsPostProcessor::GetValuesAtNode(
    const Element& element, size_t component_index) const {
  return element.GetSolutionFromDofs(component_index);
}

}  // namespace ffea