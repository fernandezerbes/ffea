#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <optional>
#include <vector>

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../geometry/node.h"
#include "../model/alias.h"
#include "../model/constitutive_model.h"
#include "./degree_of_freedom.h"
#include "./integration_point.h"

namespace ffea {

struct ElementSystem {
  std::optional<Matrix<double>> mass_matrix;
  std::optional<Matrix<double>> damping_matrix;
  std::optional<Matrix<double>> stiffness_matrix;
  std::optional<Vector<double>> rhs_vector;
};

class Element {
 public:
  Element(GeometricEntity &geometric_entity,
          const std::vector<DegreeOfFreedom *> &dofs,
          const IntegrationPointsGroup &integration_points);

  GeometricEntityType geometric_entity_type() const;
  size_t number_of_nodes() const;
  Coordinates &node_coords(size_t node_idx) const;
  std::vector<size_t> node_tags() const;
  size_t node_tag(size_t local_node_idx) const;
  size_t number_of_dofs() const;
  size_t dofs_per_node() const;
  std::vector<DegreeOfFreedom *> dofs() const;
  std::vector<size_t> dof_tags() const;

  void SetSparsity(MatrixEntries<double> &nonzero_entries) const;
  void ProcessOverDomain(const ConstitutiveModel &constitutive_model,
                         Integrand integrand, ConditionFunction source,
                         CSRMatrix<double> &global_stiffness,
                         Vector<double> &global_rhs) const;
  void ProcessOverBoundary(ConditionFunction load, ConditionFunction radiation,
                           CSRMatrix<double> &global_stiffness,
                           Vector<double> &global_rhs) const;
  Vector<double> ExtractSolution() const;
  void AddNodalValues(ValuesProcessor values_processor,
                      std::vector<ffea::NodalValuesGroup> &raw_values) const;

 private:
  Matrix<double> EvaluateShapeFunctions(
      const Coordinates &local_coords,
      DerivativeOrder order = DerivativeOrder::kZeroth) const;
  Matrix<double> EvaluateJacobian(const Coordinates &local_coords,
                                  const Matrix<double> &dN_local) const;
  Vector<double> EvaluateNormalVector(const Coordinates &local_coords) const;
  double EvaluateDifferential(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(const Matrix<double> &N_at_point) const;
  void AddLoadContribution(const std::vector<double> &load_vector,
                           const Matrix<double> &N, double weight,
                           double differential, ElementSystem &system) const;
  void AddRadiationContribution(double radiation, const Matrix<double> &N,
                                double weight, double differential,
                                ElementSystem &system) const;
  void Scatter(const ElementSystem &system, CSRMatrix<double> &global_stiffness,
               Vector<double> &global_rhs) const;

  GeometricEntity &entity_;
  std::vector<DegreeOfFreedom *> dofs_;
  const IntegrationPointsGroup &ips_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
