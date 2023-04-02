#ifndef FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
#define FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_

#include <vector>

#include "../alias.h"
#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"
#include "../geometry/node.h"
#include "../model/constitutive_model.h"
#include "./degree_of_freedom.h"
#include "./integration_point.h"

namespace ffea {

class Element {
 public:
  Element(GeometricEntity &geometric_entity, std::vector<DegreeOfFreedom *> dofs,
          const IntegrationPointsGroup &ips);

  GeometricEntityType geometric_entity_type() const;
  size_t number_of_nodes() const;
  Coordinates &node_coords(size_t node_idx) const;
  std::vector<size_t> node_tags() const;
  size_t node_tag(size_t local_node_idx) const;
  size_t number_of_dofs() const;
  size_t dofs_per_node() const;
  size_t number_of_integration_points() const;

  std::vector<DegreeOfFreedom *> dofs() const;
  std::vector<size_t> dof_tags() const;
  const IntegrationPointsGroup &integration_points() const;
  double integration_point_weight(size_t integration_point_idx) const;
  Coordinates global_coords(size_t integration_point_idx) const;
  Matrix<double> EvaluateGlobalShapeFunctionsDerivatives(size_t integration_point_idx) const;
  void SetSparsity(MatrixEntries<double> &nonzero_entries) const;
  Vector<double> ExtractSolution() const;
  void AddNodalValues(const ValuesProcessor &values_processor,
                      std::vector<ffea::NodalValuesGroup> &raw_values) const;
  Matrix<double> EvaluateShapeFunctions(size_t integration_point_idx,
                                        DerivativeOrder order = DerivativeOrder::kZeroth) const;
  Matrix<double> EvaluateShapeFunctions(const Coordinates &local_coords,
                                        DerivativeOrder order = DerivativeOrder::kZeroth) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(const Matrix<double> &N_at_point) const;
  double EvaluateDifferential(size_t integration_point_idx) const;
  Matrix<double> EvaluateJacobian(const Coordinates &local_coords,
                                  const Matrix<double> &dN_local) const;
  void ResetCache();

 private:
  Vector<double> EvaluateNormalVector(const Coordinates &local_coords) const;

  GeometricEntity &entity_;
  std::vector<DegreeOfFreedom *> dofs_;
  const IntegrationPointsGroup &ips_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_ELEMENT_H_
