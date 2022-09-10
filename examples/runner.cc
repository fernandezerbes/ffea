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

  // TODO Review responsibilities in model and analysis
  // TODO Make member variables private in model
  // TODO See if Analysis and Model can be merged
  // TODO Add differential operator and constitutive model as a reference in the
  // elements (and in the element factories)
  // TODO Reuse all possible values of shape functions, jacobians, etc. during
  // system integration
  // TODO Order element types in case statements
  // TODO Complete element factories for all element types
  // TODO Add source term
  // TODO Remove dependency of Eigen
  // TODO Add caching of differential operator, shape functions, etc.
  // TODO Create analysis builders
  // TODO Review function signatures and check constness, encapsulation, etc
  // TODO Add unit-tests
  // TODO Add smoke-tests
  // TODO Add capabilities for geometric non-linearities (non linear analysis,
  // NR Solver, etc)
  // TODO Add multithreading
  // TODO Add MPI

  // ********************** MESH **********************
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
  ffea::LinearElasticConstitutiveModel2D constitutive_model(youngs_modulus,
                                                            poisson_ratio);

  // ********************** DIFFERENTIAL OPERATOR **********************
  auto differential_operator = ffea::StrainDisplacementOperator2D();

  // ********************** MODEL **********************
  auto body_load =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  ffea::Model model(mesh);
  model.AddComputationalDomain(surface_group_name, constitutive_model,
                               differential_operator, body_load);

  // ********************** BOUNDARY CONDITIONS **********************
  auto load_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{1.0, 0.0};
    return load;
  };
  model.AddNeumannBoundaryCondition(neumann_group_name, load_function);

  auto boundary_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };
  std::unordered_set<size_t> directions_to_consider = {0, 1};
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
  // writer.WriteQuad("ffea_output_quad.vtk");
  writer.WriteTria("ffea_output_tria.vtk");

  return 0;
}
