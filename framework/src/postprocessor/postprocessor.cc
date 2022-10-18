#include "../../inc/postprocessor/postprocessor.h"

#include <numeric>
namespace ffea {

PostProcessor::PostProcessor(std::string variable_name, size_t values_per_node,
                             const Mesh &mesh)
    : mesh_(mesh),
      variable_name_(variable_name),
      values_per_node_(values_per_node) {}

std::string PostProcessor::variable_name() const { return variable_name_; }

size_t PostProcessor::values_per_node() const { return values_per_node_; }

PrimaryVariablePostProcessor::PrimaryVariablePostProcessor(
    std::string variable_name, size_t values_per_node, const Mesh &mesh)
    : PostProcessor(variable_name, values_per_node, mesh) {}

std::vector<double> PrimaryVariablePostProcessor::Process(
    const std::string &group_name) const {
  const auto &dofs = mesh_.GetElementGroupDofs(group_name);
  std::vector<double> values(dofs.size());
  for (const auto &dof : dofs) {
    values[dof->local_id()] = dof->value();
  }
  return values;
}

DerivedVariableProcessor::DerivedVariableProcessor(
    std::string variable_name, size_t values_per_node, const Mesh &mesh,
    ValuesProcessor quantity_processor)
    : PostProcessor(variable_name, values_per_node, mesh),
      values_processor_(quantity_processor) {}

std::vector<double> DerivedVariableProcessor::Process(
    const std::string &group_name) const {
  const auto raw_values = GetNodalValuesOfAllContributingElements(group_name);
  const auto avg_values = AverageNodalValues(raw_values);
  return avg_values;
}

std::vector<ffea::NodalValuesGroup>
DerivedVariableProcessor::GetNodalValuesOfAllContributingElements(
    const std::string &group_name) const {
  const auto number_of_nodes = mesh_.number_of_nodes(group_name);
  std::vector<ffea::NodalValuesGroup> raw_values(number_of_nodes);

  for (const auto &element : mesh_.GetElementGroup(group_name)) {
    element.AddNodalValues(values_processor_, raw_values);
  }

  return raw_values;
}

std::vector<double> DerivedVariableProcessor::AverageNodalValues(
    const std::vector<ffea::NodalValuesGroup> &raw_values) const {
  size_t number_of_values = values_per_node() * raw_values.size();
  std::vector<double> avg_values(number_of_values, 0.0);

  for (size_t node_idx = 0; node_idx < raw_values.size(); node_idx++) {
    const auto &values_group = raw_values[node_idx];
    const auto &group_size = values_group.size();
    for (const auto &values : values_group) {
      for (size_t value_idx = 0; value_idx < values_per_node(); value_idx++) {
        const auto &global_value_idx = node_idx * values_per_node() + value_idx;
        avg_values[global_value_idx] +=
            values[value_idx] / static_cast<double>(group_size);
      }
    }
  }

  return avg_values;
}

namespace utilities {

PrimaryVariablePostProcessor MakeDisplacementProcessor2D(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Displacement", 2, mesh);
}

PrimaryVariablePostProcessor MakeDisplacementProcessor3D(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Displacement", 3, mesh);
}

PrimaryVariablePostProcessor MakeTemperatureProcessor(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Temperature", 1, mesh);
}

DerivedVariableProcessor MakeElasticStrainProcessor(
    size_t values_per_node, const Mesh &mesh, DifferentialOperator B_operator) {
  ValuesProcessor strain_processor =
      [B_operator](const Eigen::VectorXd &solution,
                   const Coordinates &global_coords,
                   const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
    const auto &B = B_operator(dN_dGlobal);
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
          const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
    const auto &B = B_operator(dN_dGlobal);
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
