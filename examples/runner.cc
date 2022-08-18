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

  const size_t number_of_elements_in_x = 10;
  const size_t number_of_elements_in_y = 40;
  const size_t number_of_nodes_in_x = number_of_elements_in_x + 1;
  const size_t number_of_nodes_in_y = number_of_elements_in_y + 1;

  const double length_in_x = 1.0;
  const double length_in_y = 10.0;

  const double dx = length_in_x / number_of_elements_in_x;
  const double dy = length_in_y / number_of_elements_in_y;
  const double origin_x = 0.0;
  const double origin_y = 0.0;

  const size_t number_of_dofs_per_node = 2;
  std::vector<ffea::Node> nodes;
  nodes.reserve(number_of_nodes_in_x * number_of_nodes_in_y);
  for (size_t j_node = 0; j_node < number_of_nodes_in_y; j_node++) {
    for (size_t i_node = 0; i_node < number_of_nodes_in_x; i_node++) {
      double x = origin_x + i_node * dx;
      double y = origin_y + j_node * dy;
      ffea::Coordinates point({x, y, 0.0});
      size_t index = j_node * number_of_nodes_in_x + i_node;
      nodes.emplace_back(index, point, number_of_dofs_per_node);
    }
  }

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

  std::vector<ffea::Element> body;
  body.reserve(number_of_elements_in_x * number_of_elements_in_y);

  for (size_t j_element = 0; j_element < number_of_elements_in_y; j_element++) {
    for (size_t i_element = 0; i_element < number_of_elements_in_x;
         i_element++) {
      size_t index = j_element * number_of_elements_in_x + i_element;
      size_t first_node_index = index + j_element;
      size_t second_node_index = first_node_index + 1;
      size_t third_node_index =
          second_node_index + number_of_nodes_in_x;  // counter-clockwise
      size_t fourth_node_index =
          first_node_index + number_of_nodes_in_x;  // counter-clockwise
      body.push_back(quad4_factory.CreateElement(
          {&nodes[first_node_index], &nodes[second_node_index],
           &nodes[third_node_index], &nodes[fourth_node_index]}));
      std::cout << "Element " << index << ", nodes " << first_node_index << " "
                << second_node_index << " " << third_node_index << " "
                << fourth_node_index << std::endl;
    }
  }

  std::vector<ffea::Element> dirichlet_boundary;
  dirichlet_boundary.reserve(number_of_elements_in_x);

  std::vector<ffea::Element> neumann_boundary;
  neumann_boundary.reserve(number_of_elements_in_x);

  for (size_t i_element = 0; i_element < number_of_elements_in_x; i_element++) {
    size_t index_bottom = i_element;
    size_t first_node_index_bottom = index_bottom;
    size_t second_node_index_bottom = first_node_index_bottom + 1;
    std::cout << "Bottom element " << index_bottom << ", nodes "
              << first_node_index_bottom << " " << second_node_index_bottom
              << std::endl;
    dirichlet_boundary.push_back(line2_factory.CreateElement(
        {&nodes[first_node_index_bottom], &nodes[second_node_index_bottom]}));

    size_t index_top =
        i_element + (number_of_nodes_in_x * (number_of_nodes_in_y - 1));
    size_t first_node_index_top = index_top;
    size_t second_node_index_top = first_node_index_top + 1;

    std::cout << "Top element " << index_top << ", nodes "
              << first_node_index_top << " " << second_node_index_top
              << std::endl;
    neumann_boundary.push_back(line2_factory.CreateElement(
        {&nodes[first_node_index_top], &nodes[second_node_index_top]}));
  }

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

  // ********************** BOUNDARY CONDITIONS **********************
  auto body_load =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.0, 0.0};
    return load;
  };

  auto load_function =
      [](const ffea::Coordinates& coordinates) -> std::vector<double> {
    std::vector<double> load{0.1, 0.0};
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

  // for (int i = 3; i >= 0; i--) {
  //   for (int j = 0; j < 4; j++) {
  //     auto index = (i * 4 + j) * number_of_dofs_per_node + 1;
  //     std::cout << solution(index) << "\t\t";
  //   }
  //   std::cout << std::endl;
  // }

  // ********************** POSTPROCESSING **********************
  for (auto& element : body_elements) {
    element.SetSolutionOnDofs(solution);
  }

  std::shared_ptr<ffea::PostProcessor> displacement_postprocessor =
      std::make_shared<ffea::DisplacementsPostProcessor>(mesh);

  std::cout << "Postprocessing..." << std::endl;
  ffea::OutputWriter writer(mesh);
  writer.RegisterPostProcessor(*displacement_postprocessor);
  writer.Write("ffea_output.vtk");

  return 0;
}