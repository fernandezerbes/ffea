#include <unordered_set>

#include "../applications/quasi_harmonic/inc/constitutive_model.h"
#include "../applications/quasi_harmonic/inc/postprocessor.h"
#include "../applications/quasi_harmonic/inc/quasi_harmonic_region.h"
#include "../framework/inc/alias.h"
#include "../framework/inc/analysis/analysis.h"
#include "../framework/inc/fileio/output_writer.h"
#include "../framework/inc/geometry/coordinates.h"
#include "../framework/inc/geometry/geometry_builder.h"
#include "../framework/inc/mesh/mesh.h"
#include "../framework/inc/mesh/mesh_builder.h"
#include "../framework/inc/model/boundary_condition.h"
#include "../framework/inc/model/model.h"
#include "../framework/inc/model/operator.h"

int main() {
  // Analysis of an aluminum heat sink maintained at 0 C on the base and with a heat flux of 0.002 W
  // / (mm^2 * C) on the exposed surfaces

  // Read geometry from msh file
  const std::string body = "body";
  const std::string exposed_surface = "exposed";
  const std::string bottom_face = "bottom";

  ffea::GeometricEntityFactory3D geometric_entity_factory;
  ffea::GeometryFromFileBuilder geometry_builder("heat_sink.msh", geometric_entity_factory);
  auto geometry = geometry_builder.Build();

  // Crete a finite element mesh with one field (temperature)
  ffea::ElementFactory element_factory(ffea::full_integration_points);
  ffea::MeshBuilder mesh_builder(geometry, element_factory);
  const size_t number_of_fields = 1;
  auto mesh = mesh_builder.Build(number_of_fields);

  // Use an isotropic constitutive model
  double conductivity = 0.200;  // W / (mm * C)
  ffea::app::IsotropicConductivityConstitutiveModel3D constitutive_model(conductivity);

  // Set up the physics in the domain
  auto heat_source = nullptr;
  auto& body_elements = mesh.element_group(body);
  auto body_domain =
      ffea::Domain(body_elements, ffea::gradient_operator, constitutive_model, heat_source);

  // Set up the Neumann boundary conditions ()
  auto radiation = [](const ffea::Coordinates& coords, ffea::Time t) -> double { return 0.0; };

  auto load_function = [radiation](const ffea::Coordinates& coords,
                                   ffea::Time t) -> std::vector<double> {
    auto heat_flux = 2.0e-3;           // W / (mm^2 * C)
    auto free_field_temperature = 30;  // C
    return {heat_flux - radiation(coords, t) * free_field_temperature};
  };

  auto& exposed_surface_elements = mesh.element_group(exposed_surface);
  auto exposed_boundary =
      ffea::app::QuasiHarmonicDomainBoundary(exposed_surface_elements, load_function, radiation);

  // Set up Dirichlet boundary conditions (Fixed displacements in all directions)
  auto enforcement_strategy = ffea::DirectEnforcementStrategy();
  auto constraint_function = [](const ffea::Coordinates& coords,
                                ffea::Time t) -> std::vector<double> {
    auto base_temperature = 0.0;  // C
    return {base_temperature};
  };
  std::unordered_set<size_t> constrained_components = {0};
  auto& fixed_temp_elements = mesh.element_group(bottom_face);
  auto fixed_boundary = ffea::EssentialBoundaryCondition(
      fixed_temp_elements, constraint_function, constrained_components, enforcement_strategy);

  // Set up the model
  ffea::Model model(mesh);
  model.AddPhysicalRegion(body_domain);
  model.AddPhysicalRegion(exposed_boundary);
  model.AddEssentialBoundaryCondition(fixed_boundary);

  // Set up displacements postprocessor
  ffea::OutputWriter writer(mesh);
  const auto temperature_postprocessor = ffea::app::MakeTemperatureProcessor(mesh);
  writer.RegisterPostProcessor(temperature_postprocessor);

  // Create the analysis and solve
  ffea::Analysis analysis(model, writer);
  analysis.Solve("heat_sink", body);

  return 0;
}
