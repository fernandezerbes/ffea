#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <array>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <memory>
#include <unordered_set>

#include "../framework/inc/analysis/analysis.h"
#include "../framework/inc/fileio/output_writer.h"
#include "../framework/inc/geometry/coordinates.h"
#include "../framework/inc/geometry/geometry_builder.h"
#include "../framework/inc/geometry/node.h"
#include "../framework/inc/math/shape_functions.h"
#include "../framework/inc/math/utils.h"
#include "../framework/inc/mesh/degree_of_freedom.h"
#include "../framework/inc/mesh/element.h"
#include "../framework/inc/mesh/integration_points_provider.h"
#include "../framework/inc/mesh/mesh.h"
#include "../framework/inc/mesh/mesh_builder.h"
#include "../framework/inc/model/boundary_condition.h"
#include "../framework/inc/model/constitutive_model.h"
#include "../framework/inc/model/model.h"
#include "../framework/inc/model/operator.h"
#include "../framework/inc/model/physics_processor.h"
#include "../framework/inc/postprocessor/postprocessor.h"

int main() {
  // ********************** MESH **********************
  auto start = std::chrono::high_resolution_clock::now();
  size_t number_of_cores = 4;
  Eigen::setNbThreads(number_of_cores);
  std::cout << "Running with " << Eigen::nbThreads() << " threads..."
            << std::endl;

  const std::string dirichlet_group_name = "dirichlet";
  const std::string neumann_group_name = "neumann";
  const std::string body_group_name = "body";
  const size_t number_of_fields = 3;

  ffea::GeometricEntityFactory3D geometric_entity_factory;

  // std::string filename = "cube.msh";
  // std::string filename = "cylinder.msh";
  std::string filename = "cylinder_quadratic.msh";
  ffea::GeometryFromFileBuilder geometry_builder(filename,
                                                 geometric_entity_factory);

  auto geometry = geometry_builder.Build();

  auto integration_points_provider =
      ffea::utilities::MakeFullIntegrationPointsProvider();
  ffea::ElementFactory element_factory(integration_points_provider);

  ffea::MeshBuilder mesh_builder(geometry);
  mesh_builder.RegisterElementFactory(body_group_name, element_factory);
  mesh_builder.RegisterElementFactory(neumann_group_name, element_factory);
  mesh_builder.RegisterElementFactory(dirichlet_group_name, element_factory);
  auto mesh = mesh_builder.Build(number_of_fields);

  // ********************** CONSTITUTIVE MODEL **********************
  double poisson_ratio = 0.3;
  double youngs_modulus = 1;
  ffea::LinearElasticConstitutiveModel3D constitutive_model(youngs_modulus,
                                                            poisson_ratio);

  // ********************** MODEL **********************
  auto body_load =
      [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0, 0.0};
    return load;
  };

  ffea::Model model(mesh);
  model.AddComputationalDomain<ffea::ElasticityDomainProcessor>(
      body_group_name, constitutive_model, body_load,
      ffea::linear_B_operator_3D);

  // ********************** BOUNDARY CONDITIONS **********************
  auto load_function =
      [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0, 1.0};
    return load;
  };

  model.AddNeumannBoundaryCondition<ffea::ElasticityBoundaryProcessor>(
      neumann_group_name, number_of_fields, load_function);

  auto boundary_function =
      [](const ffea::Coordinates& coords) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0, 0.0};
    return load;
  };
  std::unordered_set<size_t> directions_to_consider = {0, 1, 2};
  model.AddDirichletBoundaryCondition(dirichlet_group_name, boundary_function,
                                      directions_to_consider);

  // ********************** ANALYSIS **********************
  ffea::Analysis analysis(model);
  analysis.Solve();

  // ********************** POSTPROCESSING **********************
  std::shared_ptr<ffea::PostProcessor> displacement_postprocessor =
      std::make_shared<ffea::DisplacementsPostProcessor>(mesh);

  std::cout << "Postprocessing..." << std::endl;
  ffea::OutputWriter writer(mesh);
  writer.RegisterPostProcessor(*displacement_postprocessor);
  // writer.WriteTetra("ffea_output_tetra_3d.vtk");
  writer.WriteTetraQuadratic("ffea_output_tetra_3d_quadratic.vtk");

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << "Ran in " << duration.count() << " ms" << std::endl;

  return 0;
}
