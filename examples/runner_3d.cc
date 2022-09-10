#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <memory>
#include <unordered_set>

#include "../framework/inc/analysis/analysis.h"
#include "../framework/inc/fileio/output_writer.h"
#include "../framework/inc/math/shape_functions.h"
#include "../framework/inc/math/utils.h"
#include "../framework/inc/mesh/coordinates.h"
#include "../framework/inc/mesh/degree_of_freedom.h"
#include "../framework/inc/mesh/element.h"
#include "../framework/inc/mesh/geometry_builder.h"
#include "../framework/inc/mesh/integration_points_provider.h"
#include "../framework/inc/mesh/mesh.h"
#include "../framework/inc/mesh/mesh_builder.h"
#include "../framework/inc/mesh/node.h"
#include "../framework/inc/model/boundary_condition.h"
#include "../framework/inc/model/constitutive_model.h"
#include "../framework/inc/model/model.h"
#include "../framework/inc/postprocessor/postprocessor.h"
#include "../framework/inc/processor/operator.h"

int main() {
  // ********************** MESH **********************
  const std::string dirichlet_group_name = "dirichlet";
  const std::string neumann_group_name = "neumann";
  const std::string surface_group_name = "surface";  // TODO Name `body`
  const size_t number_of_fields = 3;

  ffea::GeometricEntityFactory3D geometric_entity_factory;
  ffea::GeometryFromFileBuilder geometry_builder("cube.msh",
                                                 geometric_entity_factory);

  auto geometry = geometry_builder.Build();

  auto integration_points_provider =
      ffea::utilities::MakeFullIntegrationPointsProvider();
  ffea::ElementFactory element_factory(integration_points_provider);

  ffea::MeshBuilder mesh_builder(geometry);
  mesh_builder.RegisterElementFactory(surface_group_name, element_factory);
  mesh_builder.RegisterElementFactory(neumann_group_name, element_factory);
  mesh_builder.RegisterElementFactory(dirichlet_group_name, element_factory);
  auto mesh = mesh_builder.Build(number_of_fields);

  // ********************** CONSTITUTIVE MODEL **********************
  double poisson_ratio = 0.3;
  double youngs_modulus = 1;
  ffea::LinearElasticConstitutiveModel3D constitutive_model(youngs_modulus,
                                                            poisson_ratio);

  // ********************** DIFFERENTIAL OPERATOR **********************
  auto differential_operator = ffea::StrainDisplacementOperator3D();

  // ********************** MODEL **********************
  auto body_load =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0, 0.0};
    return load;
  };

  ffea::Model model(mesh, constitutive_model, differential_operator, body_load);

  // ********************** BOUNDARY CONDITIONS **********************
  auto load_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{1.0, 0.0, 0.0};
    return load;
  };
  model.AddNeumannBoundaryCondition(neumann_group_name, load_function);

  auto boundary_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
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
  writer.WriteTetra("ffea_output_tetra_3d.vtk");

  return 0;
}
