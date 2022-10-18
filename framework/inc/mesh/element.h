#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <optional>
#include <vector>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../geometry/node.h"
#include "../model/constitutive_model.h"
#include "../model/types.h"
#include "./degree_of_freedom.h"
#include "./integration_point.h"

namespace ffea {

struct ElementSystem {
  std::optional<Eigen::MatrixXd> mass_matrix;
  std::optional<Eigen::MatrixXd> damping_matrix;
  std::optional<Eigen::MatrixXd> stiffness_matrix;
  std::optional<Eigen::VectorXd> rhs_vector;
};

class Element {
 public:
  Element(GeometricEntity &geometric_entity,
          const std::vector<DegreeOfFreedom *> &dofs,
          const IntegrationPointsGroup &integration_points);

  GeometricEntityType GetGeometricEntityType() const;
  std::vector<size_t> GetDofTags() const;
  size_t GetNumberOfDofs() const;
  size_t number_of_nodes() const;
  size_t GetNumberOfDofsPerNode() const;
  Coordinates &GetCoordinatesOfNode(size_t node_idx) const;
  void ProcessOverDomain(const ConstitutiveModel &constitutive_model,
                         Integrand integrand, ConditionFunction source,
                         Eigen::MatrixXd &global_stiffness,
                         Eigen::VectorXd &global_rhs) const;
  void ProcessOverBoundary(ConditionFunction load, ConditionFunction radiation,
                           Eigen::MatrixXd &global_stiffness,
                           Eigen::VectorXd &global_rhs) const;
  Eigen::VectorXd GetSolution() const;
  size_t GetNodeTag(size_t local_node_idx) const;
  void AddNodalValues(ValuesProcessor values_processor,
                      std::vector<ffea::NodalValuesGroup> &raw_values) const;
  std::vector<DegreeOfFreedom *> dofs() const;
  std::vector<size_t> GetNodeTags() const;

 private:
  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coords,
      DerivativeOrder derivative_order = DerivativeOrder::kZeroth) const;
  Eigen::MatrixXd EvaluateJacobian(
      const Coordinates &local_coords,
      const Eigen::MatrixXd &shape_functions_derivatives) const;
  Eigen::VectorXd EvaluateNormalVector(const Coordinates &local_coords) const;
  double EvaluateDifferential(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(
      const Eigen::MatrixXd &shape_functions_at_point) const;
  void AddLoadContribution(const std::vector<double> &load_vector,
                           const Eigen::MatrixXd &N, double weight,
                           double differential, ElementSystem &system) const;
  void AddRadiationContribution(double radiation, const Eigen::MatrixXd &N,
                                double weight, double differential,
                                ElementSystem &system) const;
  void Scatter(const ElementSystem &element_system,
               Eigen::MatrixXd &global_stiffness,
               Eigen::VectorXd &global_rhs) const;

  GeometricEntity &geometric_entity_;
  std::vector<DegreeOfFreedom *> dofs_;
  const IntegrationPointsGroup &integration_points_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
