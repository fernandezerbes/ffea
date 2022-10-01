#include "../../inc/postprocessor/postprocessor.h"

namespace ffea {

DisplacementsPostProcessor::DisplacementsPostProcessor(const Mesh& mesh)
    : mesh_(mesh) {}

Eigen::VectorXd DisplacementsPostProcessor::GetValuesAtNode(
    const Element& element, size_t component_idx) const {
  return element.GetSolutionFromDofs(component_idx);
}

}  // namespace ffea