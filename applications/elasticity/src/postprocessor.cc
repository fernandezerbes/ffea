#include "../inc/postprocessor.h"

namespace ffea {

namespace utilities {

PrimaryVariablePostProcessor MakeDisplacementProcessor2D(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Displacement", 2, mesh);
}

PrimaryVariablePostProcessor MakeDisplacementProcessor3D(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Displacement", 3, mesh);
}

DerivedVariableProcessor MakeElasticStrainProcessor(
    size_t values_per_node, const Mesh &mesh, DifferentialOperator B_operator) {
  ValuesProcessor strain_processor =
      [B_operator](const Eigen::VectorXd &solution,
                   const Coordinates &global_coords,
                   const Eigen::MatrixXd &dN_global) -> Eigen::MatrixXd {
    const auto &B = B_operator(dN_global);
    return B * solution;
  };

  return DerivedVariableProcessor("Strain", values_per_node, mesh,
                                  strain_processor);
}

DerivedVariableProcessor MakeElasticStrainProcessor2D(const Mesh &mesh) {
  return MakeElasticStrainProcessor(3, mesh, linear_B_operator_2D);
}

DerivedVariableProcessor MakeElasticStrainProcessor3D(const Mesh &mesh) {
  return MakeElasticStrainProcessor(6, mesh, linear_B_operator_3D);
}

DerivedVariableProcessor MakeElasticStressProcessor(
    size_t values_per_node, const Mesh &mesh,
    const ConstitutiveModel &constitutive_model,
    DifferentialOperator B_operator) {
  ValuesProcessor stress_processor =
      [&constitutive_model, B_operator](
          const Eigen::VectorXd &solution, const Coordinates &global_coords,
          const Eigen::MatrixXd &dN_global) -> Eigen::MatrixXd {
    const auto &B = B_operator(dN_global);
    const auto &C = constitutive_model.Evaluate(global_coords);
    return C * B * solution;
  };

  return DerivedVariableProcessor("Stress", values_per_node, mesh,
                                  stress_processor);
}

DerivedVariableProcessor MakeElasticStressProcessor2D(
    const Mesh &mesh, const ConstitutiveModel &constitutive_model) {
  return MakeElasticStressProcessor(3, mesh, constitutive_model,
                                    linear_B_operator_2D);
}

DerivedVariableProcessor MakeElasticStressProcessor3D(
    const Mesh &mesh, const ConstitutiveModel &constitutive_model) {
  return MakeElasticStressProcessor(6, mesh, constitutive_model,
                                    linear_B_operator_2D);
}

}  // namespace utilities

}  // namespace ffea
