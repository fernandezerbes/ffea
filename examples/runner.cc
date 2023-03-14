#include <chrono>
#include <iostream>
#include <memory>
#include <unordered_set>

#include "../applications/elasticity/inc/constitutive_model.h"
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
  /*     0         1         2
    12--------13--------14--------15
    |         |         |         |
    |    6    |    7    |    8    |
    |         |         |         |
    8---------9---------10--------11
    |         |         |         |
    |    3    |    4    |    5    |
    |         |         |         |
    4---------5---------6---------7
    |         |         |         |
    |    0    |    1    |    2    |
    |         |         |         |
    0---------1---------2---------3

  */

  // TODO Add unit-tests
  // TODO Use ONEmkl and PARDISO Solver
  // TODO Add check for mesh file format version
  // TODO Check results for axial case
  // TODO Reuse all possible values of shape functions, jacobians, etc. during
  // system integration
  // TODO See if protected virtual classes can be made private
  // TODO Add system_mass matrix contribution
  // TODO Remove dependency of Eigen
  // TODO Create analysis builders
  // TODO Review function signatures and check constness, encapsulation, etc
  // TODO Add capabilities for geometric non-linearities (non linear analysis,
  // TODO Add smoke-tests
  // NR Solver, etc)
  // TODO Add multithreading
  // TODO Add MPI

  // ********************** MESH **********************
  auto start = std::chrono::high_resolution_clock::now();
  size_t number_of_cores = 4;
  Eigen::setNbThreads(number_of_cores);
  std::cout << "Running with " << Eigen::nbThreads() << " threads..." << std::endl;

  const size_t number_of_fields = 2;
  const std::string dirichlet_group_name = "dirichlet";
  const std::string neumann_group_name = "neumann";
  const std::string surface_group_name = "surface";

  ffea::GeometricEntityFactory2D geometric_entity_factory;
  // ffea::GeometryFromFileBuilder geometry_builder("LShapedStructure.msh",
  //                                                geometric_entity_factory);
  ffea::GeometryFromFileBuilder geometry_builder("LShapedStructureTria.msh",
                                                 geometric_entity_factory);

  auto geometry = geometry_builder.Build();

  ffea::ElementFactory element_factory(ffea::full_integration_points);

  ffea::MeshBuilder mesh_builder(geometry);
  mesh_builder.RegisterElementFactory(surface_group_name, element_factory);
  mesh_builder.RegisterElementFactory(neumann_group_name, element_factory);
  mesh_builder.RegisterElementFactory(dirichlet_group_name, element_factory);
  auto mesh = mesh_builder.Build(number_of_fields);

  // ********************** CONSTITUTIVE MODEL **********************
  double poisson_ratio = 0.3;
  double youngs_modulus = 1;
  ffea::LinearElasticConstitutiveModel2D constitutive_model(youngs_modulus, poisson_ratio);
  // ********************** MODEL **********************
  auto body_load = [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  ffea::Model model(mesh);
  model.AddComputationalDomain(surface_group_name, constitutive_model,
                               ffea::elasticity_integrand_2D, body_load);

  // ********************** BOUNDARY CONDITIONS **********************
  auto load_function = [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{1.0, 0.0};
    return load;
  };

  model.AddNaturalBoundaryCondition(neumann_group_name, load_function, nullptr);

  auto boundary_function = [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };
  std::unordered_set<size_t> directions_to_consider = {0, 1};
  model.AddEssentialBoundaryCondition(dirichlet_group_name, boundary_function,
                                      directions_to_consider);

  // ********************** ANALYSIS **********************
  ffea::Analysis analysis(model);
  analysis.Solve();

  // ********************** POSTPROCESSING **********************
  const auto& displacement_postprocessor = ffea::utilities::MakeDisplacementProcessor2D(mesh);

  std::cout << "Postprocessing..." << std::endl;
  ffea::OutputWriter writer(mesh);
  writer.RegisterPostProcessor(displacement_postprocessor);
  writer.Write("ffea_output_tria_vtu11.vtu", surface_group_name);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = stop - start;

  std::cout << "Ran in " << duration.count() << " ns" << std::endl;

  return 0;
}
