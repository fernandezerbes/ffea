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
#include "../framework/inc/mesh/integration_point.h"
#include "../framework/inc/mesh/mesh.h"
#include "../framework/inc/mesh/mesh_builder.h"
#include "../framework/inc/mesh/node.h"
#include "../framework/inc/model/boundary_condition.h"
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

  // TODO Refactor quadrature
  // TODO Add shape functions for other elements (take from Felippa book)
  // TODO Add class for constitutive model
  // TODO Make member variables private in model
  // TODO Review responsibilities in model and analysis
  // TODO Review dimensions (local, global) and where we need to store them
  // TODO Remove dependency of Eigen
  // TODO Add caching of differential operator, shape functions, etc.
  // TODO Review element factories to see if something better can be done
  // TODO Create analysis builders
  // TODO Review function signatures and check constness, encapsulation, etc
  // TODO Add unit-tests
  // TODO Add smoke-tests
  // TODO Add capabilities for geometric non-linearities (non linear analysis,
  // NR Solver, etc)
  // TODO Add multithreading
  // TODO Add MPI

  // ********************** MESH **********************
  const size_t number_of_elements_in_x = 20;
  const size_t number_of_elements_in_y = 20;
  const size_t number_of_dofs_per_node = 2;
  const double length_in_x = 1.0;
  const double length_in_y = 3.0;
  // ffea::CartesianMeshBuilder mesh_builder(length_in_x, length_in_y,
  //                                         number_of_elements_in_x,
  //                                         number_of_elements_in_y);

  // auto mesh = mesh_builder.Build(number_of_dofs_per_node);

  // ffea::MeshFromFileBuilder mesh_builder_from_file("rectangle.msh");
  ffea::MeshFromFileBuilder mesh_builder_from_file("LShapedStructure.msh");
  auto mesh = mesh_builder_from_file.Build(number_of_dofs_per_node);

  // ********************** CONSTITUTIVE MODEL **********************
  double nu = 0.3;
  double E = 1;
  double factor = E / (1 - nu * nu);
  Eigen::MatrixXd constitutive_matrix(3, 3);
  constitutive_matrix(0, 0) = factor;
  constitutive_matrix(0, 1) = factor * nu;
  constitutive_matrix(1, 0) = constitutive_matrix(0, 1);
  constitutive_matrix(1, 1) = factor;
  constitutive_matrix(2, 2) = (1 - nu) * factor;

  // ********************** DIFFERENTIAL OPERATOR **********************
  auto differential_operator = ffea::StrainDisplacementOperator2D();

  // ********************** MODEL **********************
  auto body_load =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  ffea::Model model(mesh, constitutive_matrix, differential_operator,
                    body_load);

  // ********************** BOUNDARY CONDITIONS **********************
  auto load_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 1.0};
    return load;
  };
  model.AddNeumannBoundaryCondition("neumann", load_function);

  auto boundary_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };
  std::unordered_set<size_t> directions_to_consider = {0, 1};
  model.AddDirichletBoundaryCondition("dirichlet", boundary_function,
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
  writer.Write("ffea_output.vtk");

  return 0;
}
