#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <eigen3/Eigen/Dense>
#include <memory>

#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./boundary_condition.h"
#include "./computational_domain.h"
#include "./constitutive_model.h"
#include "./physics_processor.h"
#include "./types.h"

namespace ffea {

class Model {
 public:
  explicit Model(Mesh &mesh);

  template <typename ProcessorType>
  void AddComputationalDomain(const std::string &domain_name,
                              const ConstitutiveModel &constitutive_model,
                              ConditionFunction source,
                              DifferentialOperator B_operator);
  template <typename ProcessorType>
  void AddNeumannBoundaryCondition(const std::string &boundary_name,
                                   size_t dimensions,
                                   ConditionFunction boundary_load);
  void AddDirichletBoundaryCondition(
      const std::string &boundary_name, ConditionFunction boundary_function,
      const std::unordered_set<size_t> &directions_to_consider,
      const EnforcementStrategy &enforcement_strategy =
          ffea::PenaltyEnforcementStrategy());
  void ProjectSolutionOnMesh(const Eigen::VectorXd &solution);
  size_t NumberOfDofs() const;
  void AddComputationalDomainsContributions(Eigen::MatrixXd &global_stiffness,
                                            Eigen::VectorXd &global_rhs) const;
  void EnforceBoundaryConditions(Eigen::MatrixXd &global_stiffness,
                                 Eigen::VectorXd &global_rhs) const;

 private:
  Mesh &mesh_;
  std::vector<ComputationalDomain> computational_domains_;
  std::vector<std::unique_ptr<BoundaryCondition>> boundary_conditions_;
};

template <typename ProcessorType>
void Model::AddComputationalDomain(const std::string &domain_name,
                                   const ConstitutiveModel &constitutive_model,
                                   ConditionFunction source,
                                   DifferentialOperator B_operator) {
  const auto &domain_elements = mesh_.GetElementGroup(domain_name);
  auto processor =
      std::make_unique<ProcessorType>(constitutive_model, source, B_operator);
  computational_domains_.emplace_back(domain_elements, std::move(processor));
}

template <typename ProcessorType>
void Model::AddNeumannBoundaryCondition(const std::string &boundary_name,
                                        size_t dimensions,
                                        ConditionFunction boundary_load) {
  const auto &boundary_elements = mesh_.GetElementGroup(boundary_name);
  auto processor = std::make_unique<ProcessorType>(dimensions, boundary_load);
  boundary_conditions_.push_back(
      std::make_unique<ffea::NeumannBoundaryCondition>(boundary_elements,
                                                       std::move(processor)));
}

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
