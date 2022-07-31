#ifndef FFEA_FRAMEWORK_INC_PROCESSOR_PHYSICS_PROCESSOR_H_
#define FFEA_FRAMEWORK_INC_PROCESSOR_PHYSICS_PROCESSOR_H_

#include <eigen3/Eigen/Dense>
#include <functional>
#include <utility>

#include "../mesh/coordinates.h"
#include "../mesh/element.h"
#include "../mesh/integration_point.h"
namespace ffea {

using LoadFunction = std::function<std::vector<double>(const Coordinates &)>;

class ElementProcessor {
 public:
  ElementProcessor(Eigen::MatrixXd constitutive_model);
  ~ElementProcessor();

  std::pair<Eigen::MatrixXd, Eigen::VectorXd> ProcessBodyElement(
      const Element &element, LoadFunction body_load_function) const;
  Eigen::VectorXd ProcessBoundaryElement(
      const Element &element, LoadFunction boundary_load_function) const;

 private:
  void AddContributionToRhs(const Element &element,
                            const IntegrationPoint &integration_point,
                            const Eigen::MatrixXd &jacobian,
                            LoadFunction load_function,
                            Eigen::VectorXd &rhs) const;

  void AddContributionToStiffness(const Element &element,
                                  const IntegrationPoint &integration_point,
                                  const Eigen::MatrixXd &jacobian,
                                  Eigen::MatrixXd &stiffness) const;

  Eigen::MatrixXd constitutive_model_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_PROCESSOR_PHYSICS_PROCESSOR_H_
