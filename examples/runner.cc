#include <chrono>
#include <iostream>
#include <memory>
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
#include "../framework/inc/math/utils.h"
#include "../framework/inc/mesh/degree_of_freedom.h"
#include "../framework/inc/mesh/element.h"
#include "../framework/inc/mesh/integration_point.h"
#include "../framework/inc/mesh/mesh.h"
#include "../framework/inc/mesh/mesh_builder.h"
#include "../framework/inc/model/boundary_condition.h"
#include "../framework/inc/model/constitutive_model.h"
#include "../framework/inc/model/model.h"
#include "../framework/inc/model/operator.h"
#include "../framework/inc/postprocessor/postprocessor.h"

int main() {
  // ********************** MESH **********************
  auto start = std::chrono::high_resolution_clock::now();
  size_t number_of_cores = 4;
  Eigen::setNbThreads(number_of_cores);
  std::cout << "Running with " << Eigen::nbThreads() << " threads..." << std::endl;

  const size_t number_of_fields = 2;
  const std::string body = "beam";
  const std::string fixed_end = "fixed-end";
  const std::string free_end = "free-end";

  ffea::GeometricEntityFactory2D geometric_entity_factory;
  ffea::GeometryFromFileBuilder geometry_builder("beam2d.msh", geometric_entity_factory);

  auto geometry = geometry_builder.Build();

  ffea::ElementFactory element_factory(ffea::full_integration_points);
  ffea::MeshBuilder mesh_builder(geometry, element_factory);
  auto mesh = mesh_builder.Build(number_of_fields);

  // ********************** CONSTITUTIVE MODEL **********************
  double poisson_ratio = 0.3;
  double youngs_modulus = 1;
  ffea::app::LinearElasticConstitutiveModel2D constitutive_model(youngs_modulus, poisson_ratio);
  // ********************** MODEL **********************
  auto body_load = [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };
  auto& body_elements = mesh.element_group(body);
  auto elastic_domain =
      ffea::Domain(body_elements, ffea::app::linear_B_operator_2D, constitutive_model, body_load);

  ffea::Model model(mesh);
  model.AddPhysicalRegion(elastic_domain);

  // ********************** BOUNDARY CONDITIONS **********************
  auto load_function = [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, -1.0};
    return load;
  };

  auto& free_edge_elements = mesh.element_group(free_end);
  auto loaded_boundary = ffea::DomainBoundary(free_edge_elements, load_function);
  model.AddPhysicalRegion(loaded_boundary);

  auto boundary_function = [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  std::unordered_set<size_t> directions_to_consider = {0, 1};
  auto& fixed_edge_elements = mesh.element_group(fixed_end);
  auto penalty_enforcement = ffea::PenaltyEnforcementStrategy();
  auto fixed_boundary = ffea::EssentialBoundaryCondition(
      fixed_edge_elements, boundary_function, directions_to_consider, penalty_enforcement);

  model.AddEssentialBoundaryCondition(fixed_boundary);

  // ********************** POSTPROCESSING **********************
  const auto& displacement_postprocessor = ffea::app::MakeDisplacementProcessor2D(mesh);

  std::cout << "Postprocessing..." << std::endl;
  ffea::OutputWriter writer(mesh);
  writer.RegisterPostProcessor(displacement_postprocessor);

  // ********************** ANALYSIS **********************
  ffea::Analysis analysis(model, writer);
  analysis.Solve("beam2d.vtu", body);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = stop - start;

  std::cout << "Ran in " << duration.count() << " ns" << std::endl;

  return 0;
}
