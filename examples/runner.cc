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

  // ********************** MESH **********************
  const size_t parametric_dimensions_1d = 1;
  const size_t parametric_dimensions_2d = 2;
  const size_t spatial_dimensions = 2;

  ffea::Coordinates point_0({0.0, 0.0, 0.0});
  ffea::Coordinates point_1({1.0, 0.0, 0.0});
  ffea::Coordinates point_2({2.0, 0.0, 0.0});
  ffea::Coordinates point_3({3.0, 0.0, 0.0});

  ffea::Coordinates point_4({0.0, 1.0, 0.0});
  ffea::Coordinates point_5({1.0, 1.0, 0.0});
  ffea::Coordinates point_6({2.0, 1.0, 0.0});
  ffea::Coordinates point_7({3.0, 1.0, 0.0});

  ffea::Coordinates point_8({0.0, 2.0, 0.0});
  ffea::Coordinates point_9({1.0, 2.0, 0.0});
  ffea::Coordinates point_10({2.0, 2.0, 0.0});
  ffea::Coordinates point_11({3.0, 2.0, 0.0});

  ffea::Coordinates point_12({0.0, 3.0, 0.0});
  ffea::Coordinates point_13({1.0, 3.0, 0.0});
  ffea::Coordinates point_14({2.0, 3.0, 0.0});
  ffea::Coordinates point_15({3.0, 3.0, 0.0});

  size_t number_of_dofs_per_node = 2;
  std::vector<ffea::Node> nodes;
  nodes.push_back(ffea::Node(0, point_0, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(1, point_1, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(2, point_2, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(3, point_3, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(4, point_4, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(5, point_5, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(6, point_6, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(7, point_7, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(8, point_8, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(9, point_9, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(10, point_10, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(11, point_11, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(12, point_12, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(13, point_13, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(14, point_14, number_of_dofs_per_node));
  nodes.push_back(ffea::Node(15, point_15, number_of_dofs_per_node));

  std::shared_ptr<ffea::ShapeFunctions> linear_1d_shape_functions =
      std::make_shared<ffea::Linear1DShapeFunctions>();
  std::shared_ptr<ffea::ShapeFunctions> linear_2d_shape_functions =
      std::make_shared<ffea::Linear2DShapeFunctions>();

  std::shared_ptr<ffea::QuadratureRule> integration_1x2_rule =
      std::make_shared<ffea::QuadratureRule1x2>();
  std::shared_ptr<ffea::QuadratureRule> integration_2x2_rule =
      std::make_shared<ffea::QuadratureRule2x2>();

  ffea::ElementFactory line2_factory(parametric_dimensions_1d,
                                     linear_1d_shape_functions,
                                     integration_1x2_rule);
  ffea::ElementFactory quad4_factory(parametric_dimensions_2d,
                                     linear_2d_shape_functions,
                                     integration_2x2_rule);

  std::vector<ffea::Element> dirichlet_boundary;
  dirichlet_boundary.push_back(
      line2_factory.CreateElement({&nodes[0], &nodes[1]}));
  dirichlet_boundary.push_back(
      line2_factory.CreateElement({&nodes[1], &nodes[2]}));
  dirichlet_boundary.push_back(
      line2_factory.CreateElement({&nodes[2], &nodes[3]}));

  std::vector<ffea::Element> neumann_boundary;
  neumann_boundary.push_back(
      line2_factory.CreateElement({&nodes[12], &nodes[13]}));
  neumann_boundary.push_back(
      line2_factory.CreateElement({&nodes[13], &nodes[14]}));
  neumann_boundary.push_back(
      line2_factory.CreateElement({&nodes[14], &nodes[15]}));

  std::vector<ffea::Element> body;
  body.push_back(quad4_factory.CreateElement(
      {&nodes[0], &nodes[1], &nodes[5], &nodes[4]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[1], &nodes[2], &nodes[6], &nodes[5]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[2], &nodes[3], &nodes[7], &nodes[6]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[4], &nodes[5], &nodes[9], &nodes[8]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[5], &nodes[6], &nodes[10], &nodes[9]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[6], &nodes[7], &nodes[11], &nodes[10]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[8], &nodes[9], &nodes[13], &nodes[12]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[9], &nodes[10], &nodes[14], &nodes[13]}));
  body.push_back(quad4_factory.CreateElement(
      {&nodes[10], &nodes[11], &nodes[15], &nodes[14]}));

  ffea::Mesh mesh(number_of_dofs_per_node, nodes);
  mesh.RegisterElementGroup(ffea::ElementGroupType::kBodyElements, "body",
                            body);
  mesh.RegisterElementGroup(ffea::ElementGroupType::kDirichletBoundaryElements,
                            "dirichlet_boundary", dirichlet_boundary);
  mesh.RegisterElementGroup(ffea::ElementGroupType::kNeumannBoundaryElements,
                            "neumann_boundary", neumann_boundary);

  auto& body_elements =
      mesh.GetElementGroup(ffea::ElementGroupType::kBodyElements, "body");
  auto& dirichlet_elements = mesh.GetElementGroup(
      ffea::ElementGroupType::kDirichletBoundaryElements, "dirichlet_boundary");
  auto& neumann_elements = mesh.GetElementGroup(
      ffea::ElementGroupType::kNeumannBoundaryElements, "neumann_boundary");

  // ********************** CONSTITUTIVE MODEL **********************
  double nu = 0.0;
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

  // ********************** BOUNDARY CONDITIONS **********************
  auto body_load =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  auto load_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{1.0, 0.0};
    return load;
  };

  auto boundary_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  std::shared_ptr<ffea::BoundaryCondition> load_on_top =
      std::make_shared<ffea::NeumannBoundaryCondition>(neumann_elements,
                                                       load_function);

  double penalty = 1.0e12;
  const auto& enforcement_strategy = ffea::PenaltyEnforcementStrategy(penalty);
  std::unordered_set<size_t> directions_to_consider = {0, 1};
  std::shared_ptr<ffea::BoundaryCondition> fixed_bottom =
      std::make_shared<ffea::DirichletBoundaryCondition>(
          dirichlet_elements, boundary_function, directions_to_consider,
          enforcement_strategy);

  std::vector<ffea::BoundaryCondition*> boundary_conditions;
  // TODO ensure that these are applied in the correct order
  boundary_conditions.push_back(load_on_top.get());
  boundary_conditions.push_back(fixed_bottom.get());

  // ********************** MODEL **********************
  ffea::Model model(mesh, constitutive_matrix, differential_operator,
                    boundary_conditions, body_load);

  // ********************** ANALYSIS **********************
  ffea::Analysis analysis(model);

  // ********************** SOLUTION **********************
  const auto& solution = analysis.Solve();

  for (int i = 3; i >= 0; i--) {
    for (int j = 0; j < 4; j++) {
      auto index = (i * 4 + j) * number_of_dofs_per_node + 1;
      std::cout << solution(index) << "\t\t";
    }
    std::cout << std::endl;
  }

  // ********************** POSTPROCESSING **********************
  for (auto &element: body_elements) {
    element.SetSolutionOnDofs(solution);
  }


  std::shared_ptr<ffea::PostProcessor> displacement_postprocessor =
      std::make_shared<ffea::DisplacementsPostProcessor>(mesh);

  ffea::OutputWriter writer(mesh);
  writer.RegisterPostProcessor(*displacement_postprocessor);
  writer.Write("ffea_output.vtk");

  return 0;
}