#include "../../inc/postprocessor/postprocessor.h"

#include <numeric>
namespace ffea {

PostProcessor::PostProcessor(std::string variable_name,
                             size_t components_per_node, const Mesh &mesh)
    : mesh_(mesh),
      variable_name_(variable_name),
      components_per_node_(components_per_node) {}

std::string PostProcessor::variable_name() const {
  return variable_name_;
}

size_t PostProcessor::components_per_node() const {
  return components_per_node_;
}

PrimaryVariablePostProcessor::PrimaryVariablePostProcessor(
    std::string variable_name, size_t components_per_node,
    const Mesh &mesh)
    : PostProcessor(variable_name, components_per_node, mesh) {}

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
    std::string variable_name, size_t components_per_node,
    const Mesh &mesh, QuantityProcessor quantity_processor)
    : PostProcessor(variable_name, components_per_node, mesh),
      quantity_processor_(quantity_processor) {}

std::vector<double> DerivedVariableProcessor::Process(
    const std::string &group_name) const {
  std::vector<std::vector<std::vector<double>>>
      nodal_values;  // <node<element<component>>>
  // Add data.reserve with the number of nodes in the element group
  const auto &elements = mesh_.GetElementGroup(group_name);
  for (const auto &element : elements) {
    element.AddNodalQuantities(quantity_processor_, nodal_values);
  }

  size_t number_of_values = components_per_node() * nodal_values.size();
  std::vector<double> avg_nodal_values(number_of_values, 0.0);
  avg_nodal_values.reserve(nodal_values.size() * components_per_node());
  for (size_t node_idx = 0; node_idx < nodal_values.size(); node_idx++) {
    const auto &nodal_values_all_contributions = nodal_values[node_idx];
    const auto &number_of_contributing_elements =
        nodal_values_all_contributions.size();
    for (const auto &element_contributions : nodal_values_all_contributions) {
      for (size_t component_idx = 0;
           component_idx < components_per_node(); component_idx++) {
        const auto &variable_idx =
            node_idx * components_per_node() + component_idx;
        avg_nodal_values[variable_idx] +=
            element_contributions[component_idx] /
            static_cast<double>(number_of_contributing_elements);
      }
    }
  }

  return avg_nodal_values;
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
    size_t components_per_node, const Mesh &mesh,
    DifferentialOperator B_operator) {
  QuantityProcessor strain_processor =
      [B_operator](const Eigen::VectorXd &solution,
                   const Coordinates &global_coords,
                   const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
    const auto &B = B_operator(dN_dGlobal);
    return B * solution;
  };

  return DerivedVariableProcessor("Strain", components_per_node, mesh,
                                  strain_processor);
}

DerivedVariableProcessor MakeElasticStrainProcessor2D(const Mesh &mesh) {
  return MakeElasticStrainProcessor(3, mesh, linear_B_operator_2D);
}

DerivedVariableProcessor MakeElasticStrainProcessor3D(const Mesh &mesh) {
  return MakeElasticStrainProcessor(6, mesh, linear_B_operator_3D);
}

DerivedVariableProcessor MakeElasticStressProcessor(
    size_t components_per_node, const Mesh &mesh,
    const ConstitutiveModel &constitutive_model,
    DifferentialOperator B_operator) {
  QuantityProcessor stress_processor =
      [&constitutive_model, B_operator](
          const Eigen::VectorXd &solution, const Coordinates &global_coords,
          const Eigen::MatrixXd &dN_dGlobal) -> Eigen::MatrixXd {
    const auto &B = B_operator(dN_dGlobal);
    const auto &C = constitutive_model.Evaluate(global_coords);
    return C * B * solution;
  };

  return DerivedVariableProcessor("Stress", components_per_node, mesh,
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