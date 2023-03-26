#include <unordered_set>

#include "../applications/elasticity/inc/constitutive_model.h"
#include "../applications/elasticity/inc/elastic_region.h"
#include "../applications/elasticity/inc/operator.h"
#include "../applications/elasticity/inc/postprocessor.h"
#include "../framework/inc/alias.h"
#include "../framework/inc/analysis/analysis.h"
#include "../framework/inc/fileio/output_writer.h"
#include "../framework/inc/geometry/coordinates.h"
#include "../framework/inc/geometry/geometry_builder.h"
#include "../framework/inc/geometry/node.h"
#include "../framework/inc/mesh/mesh.h"
#include "../framework/inc/mesh/mesh_builder.h"
#include "../framework/inc/model/boundary_condition.h"
#include "../framework/inc/model/model.h"

int main() {
  // Analysis of a 1-meter cantilever IPB200 steel beam with a 1000 N downward force on the free end

  // Read geometry from msh file
  const std::string beam = "beam";
  const std::string fixed_face = "fixed";
  const std::string loaded_face = "load";

  ffea::GeometricEntityFactory3D geometric_entity_factory;
  ffea::GeometryFromFileBuilder geometry_builder("ipb200.msh", geometric_entity_factory);
  auto geometry = geometry_builder.Build();

  // Crete a finite element mesh with 3 displacement fields (u, v, w)
  ffea::ElementFactory element_factory(ffea::full_integration_points);
  ffea::MeshBuilder mesh_builder(geometry, element_factory);
  const size_t number_of_fields = 3;
  auto mesh = mesh_builder.Build(number_of_fields);

  // Use a linear-elastic constitutive model
  double poisson_ratio = 0.3;
  double youngs_modulus = 2.1e5;
  ffea::app::LinearElasticConstitutiveModel3D constitutive_model(youngs_modulus, poisson_ratio);

  // Set up the physics in the domain
  auto body_load = nullptr;  // No body load (gravity)
  auto& body_elements = mesh.element_group(beam);
  auto elastic_domain =
      ffea::Domain(body_elements, ffea::app::linear_B_operator_3D, constitutive_model, body_load);

  // Set up the Neumann boundary conditions (Load F = 1000 in -y direction)
  auto load_function = [](const ffea::Coordinates& coords, ffea::Time t) -> std::vector<double> {
    return {0.0, -1000.0, 0.0};
  };

  auto& loaded_face_elements = mesh.element_group(loaded_face);
  auto loaded_boundary = ffea::DomainBoundary(loaded_face_elements, load_function);

  // Set up Dirichlet boundary conditions (Fixed displacements in all directions)
  auto enforcement_strategy = ffea::DirectEnforcementStrategy();
  auto constraint_function = [](const ffea::Coordinates& coords,
                                ffea::Time t) -> std::vector<double> {
    return {0.0, 0.0, 0.0};
  };
  std::unordered_set<size_t> constrained_directions = {0, 1, 2};
  auto& fixed_face_elements = mesh.element_group(fixed_face);
  auto fixed_boundary = ffea::EssentialBoundaryCondition(
      fixed_face_elements, constraint_function, constrained_directions, enforcement_strategy);

  // Set up the model
  ffea::Model model(mesh);
  model.AddPhysicalRegion(elastic_domain);
  model.AddPhysicalRegion(loaded_boundary);
  model.AddEssentialBoundaryCondition(fixed_boundary);

  // Set up displacements postprocessor
  ffea::OutputWriter writer(mesh);
  const auto displacement_postprocessor = ffea::app::MakeDisplacementProcessor3D(mesh);
  writer.RegisterPostProcessor(displacement_postprocessor);

  // Create the analysis and solve
  ffea::Analysis analysis(model, writer);
  analysis.Solve("ipb200", {beam});

  return 0;
}
