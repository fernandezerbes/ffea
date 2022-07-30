#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <memory>

#include "../framework/inc/math/CustomDenseMatrix.h"
#include "../framework/inc/math/Utils.h"
#include "../framework/inc/mesh/Coordinates.h"
#include "../framework/inc/mesh/DegreeOfFreedom.h"
#include "../framework/inc/mesh/IntegrationPoint.h"
#include "../framework/inc/mesh/Line2.h"
#include "../framework/inc/mesh/Mesh.h"
#include "../framework/inc/mesh/Node.h"
#include "../framework/inc/mesh/Quad4.h"

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

  ffea::Coordinates point_0(0.0, 0.0, 0.0);
  ffea::Coordinates point_1(1.0, 0.0, 0.0);
  ffea::Coordinates point_2(2.0, 0.0, 0.0);
  ffea::Coordinates point_3(3.0, 0.0, 0.0);

  ffea::Coordinates point_4(0.0, 1.0, 0.0);
  ffea::Coordinates point_5(1.0, 1.0, 0.0);
  ffea::Coordinates point_6(2.0, 1.0, 0.0);
  ffea::Coordinates point_7(3.0, 1.0, 0.0);

  ffea::Coordinates point_8(0.0, 2.0, 0.0);
  ffea::Coordinates point_9(1.0, 2.0, 0.0);
  ffea::Coordinates point_10(2.0, 2.0, 0.0);
  ffea::Coordinates point_11(3.0, 2.0, 0.0);

  ffea::Coordinates point_12(0.0, 3.0, 0.0);
  ffea::Coordinates point_13(1.0, 3.0, 0.0);
  ffea::Coordinates point_14(2.0, 3.0, 0.0);
  ffea::Coordinates point_15(3.0, 3.0, 0.0);

  short int number_of_dofs_per_node = 2;
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

  std::vector<ffea::Element*> dirichlet_boundary;
  dirichlet_boundary.push_back(new ffea::Line2({&nodes[0], &nodes[1]}));
  dirichlet_boundary.push_back(new ffea::Line2({&nodes[1], &nodes[2]}));
  dirichlet_boundary.push_back(new ffea::Line2({&nodes[2], &nodes[3]}));

  std::vector<ffea::Element*> neumann_boundary;
  neumann_boundary.push_back(new ffea::Line2({&nodes[12], &nodes[13]}));
  neumann_boundary.push_back(new ffea::Line2({&nodes[13], &nodes[14]}));
  neumann_boundary.push_back(new ffea::Line2({&nodes[14], &nodes[15]}));

  std::vector<ffea::Element*> body;
  body.push_back(new ffea::Quad4({&nodes[0], &nodes[1], &nodes[5], &nodes[4]}));
  body.push_back(new ffea::Quad4({&nodes[1], &nodes[2], &nodes[6], &nodes[5]}));
  body.push_back(new ffea::Quad4({&nodes[2], &nodes[3], &nodes[7], &nodes[6]}));
  body.push_back(new ffea::Quad4({&nodes[4], &nodes[5], &nodes[9], &nodes[8]}));
  body.push_back(
      new ffea::Quad4({&nodes[5], &nodes[6], &nodes[10], &nodes[9]}));
  body.push_back(
      new ffea::Quad4({&nodes[6], &nodes[7], &nodes[11], &nodes[10]}));
  body.push_back(
      new ffea::Quad4({&nodes[8], &nodes[9], &nodes[13], &nodes[12]}));
  body.push_back(
      new ffea::Quad4({&nodes[9], &nodes[10], &nodes[14], &nodes[13]}));
  body.push_back(
      new ffea::Quad4({&nodes[10], &nodes[11], &nodes[15], &nodes[14]}));

  std::vector<ffea::IntegrationPoint> quad_integration_points;
  quad_integration_points.push_back(ffea::IntegrationPoint(
      {-0.5773502691896257, -0.5773502691896257, 0.0}, 1.0));
  quad_integration_points.push_back(ffea::IntegrationPoint(
      {0.5773502691896257, -0.5773502691896257, 0.0}, 1.0));
  quad_integration_points.push_back(ffea::IntegrationPoint(
      {-0.5773502691896257, 0.5773502691896257, 0.0}, 1.0));
  quad_integration_points.push_back(ffea::IntegrationPoint(
      {0.5773502691896257, 0.5773502691896257, 0.0}, 1.0));

  std::vector<ffea::IntegrationPoint> line_integration_points;
  line_integration_points.push_back(
      ffea::IntegrationPoint({-0.5773502691896257, 0.0, 0.0}, 1.0));
  line_integration_points.push_back(
      ffea::IntegrationPoint({0.5773502691896257, 0.0, 0.0}, 1.0));

  ffea::Mesh mesh(number_of_dofs_per_node, nodes);
  // mesh.RegisterElementGroup("dirichlet_boundary", dirichlet_boundary);
  // mesh.RegisterElementGroup("neumann_boundary", neumann_boundary);
  // mesh.RegisterElementGroup("body", body);
  mesh.dirichlet_boundary_ = dirichlet_boundary;
  mesh.neumann_boundary_ = neumann_boundary;
  mesh.body_ = body;

  // Constitutive matrix
  double nu = 0.3;
  double E = 210e9;
  double factor = E / (1 - nu * nu);
  Eigen::MatrixXd C(3, 3);
  C(0, 0) = factor;
  C(0, 1) = factor * nu;
  C(1, 0) = C(0, 1);
  C(1, 1) = factor;
  C(2, 2) = (1 - nu) * factor;

  // Stiffness matrix
  auto number_of_dofs = mesh.number_of_dofs();
  Eigen::SparseMatrix<double, Eigen::RowMajor> global_K(number_of_dofs,
                                                        number_of_dofs);
  global_K.setZero();
  std::vector<Eigen::Triplet<double>> coefficients;


  // const auto& elements = mesh.GetElementGroup("body");

  for (const auto& element : mesh.body_) {
    size_t number_of_nodes = element->number_of_nodes();
    size_t number_of_dofs = element->number_of_dofs();
    Eigen::MatrixXd element_K(number_of_dofs, number_of_dofs);
    const auto& element_nodes_coordinates = element->node_coordinates();
    for (const auto& integration_point : quad_integration_points) {
      const auto& local_coordinates = integration_point.local_coordinates();
      const auto& jacobian = element->EvaluateJacobian(local_coordinates);
      const auto& dN_local =
          element->EvaluateShapeFunctionsDerivatives(local_coordinates);
      const auto& dN_global = jacobian.inverse() * dN_local;
      Eigen::MatrixXd B(3, number_of_dofs);
      for (size_t node_index = 0; node_index < number_of_nodes; node_index++) {
        auto first_dof_index = number_of_dofs_per_node * node_index;
        auto second_dof_index = first_dof_index + 1;
        B(0, first_dof_index) = dN_global(0, node_index);
        B(1, second_dof_index) = dN_global(1, node_index);
        B(2, first_dof_index) = dN_global(1, node_index);
        B(2, second_dof_index) = dN_global(0, node_index);
      }
      element_K += B.transpose() * C * B * jacobian.determinant() *
                   integration_point.weight();
    }

    // Scatter coefficients
    auto nodes = element->nodes();
    for (size_t i_node = 0; i_node < nodes.size(); i_node++) {
      auto number_of_dofs = nodes[i_node]->number_of_dofs();
      for (size_t j_node = 0; j_node < nodes.size(); j_node++) {
        for (size_t i_dof = 0; i_dof < number_of_dofs; i_dof++) {
          for (size_t j_dof = 0; j_dof < number_of_dofs; j_dof++) {
            auto local_row_index = i_node * number_of_dofs_per_node + i_dof;
            auto local_col_index = j_node * number_of_dofs_per_node + j_dof;
            auto global_row_index =
                number_of_dofs_per_node * nodes[i_node]->id() + i_dof;
            auto global_col_index =
                number_of_dofs_per_node * nodes[j_node]->id() + j_dof;

            coefficients.push_back(Eigen::Triplet<double>(
                global_row_index, global_col_index,
                element_K(local_row_index, local_col_index)));
            // Add body load contribution // add weights in integration
          }
        }
      }
    }
  }

  global_K.setFromTriplets(coefficients.begin(), coefficients.end());
  global_K.makeCompressed();

  std::cout << "global_K\n" << global_K << std::endl;

  // Load vector
  Eigen::VectorXd global_rhs(mesh.number_of_dofs());
  global_rhs.setZero();

  auto load_function = [](const ffea::Coordinates& coordinates) {
    Eigen::VectorXd load(2);
    load(0) = 0.0;
    load(1) = 1e8;
    return load;
  };

  for (const auto& element : mesh.neumann_boundary_) {
    size_t number_of_dofs = element->number_of_dofs();
    Eigen::VectorXd element_rhs(number_of_dofs);

    Eigen::VectorXd nodal_force_vector(number_of_dofs);  // {f1x, f1y, f2x, f2y}
    auto nodes = element->nodes();
    size_t number_of_nodes = element->number_of_nodes();
    for (size_t node_index = 0; node_index < number_of_nodes; node_index++) {
      auto& node = nodes[node_index];
      auto load_at_node = load_function(node->coordinates());
      auto first_dof_index = number_of_dofs_per_node * node_index;
      auto second_dof_index = first_dof_index + 1;
      nodal_force_vector(first_dof_index) = load_at_node(0);
      nodal_force_vector(second_dof_index) = load_at_node(1);
    }

    for (const auto& integration_point : line_integration_points) {
      const auto& local_coordinates = integration_point.local_coordinates();
      const auto& N = element->EvaluateShapeFunctions(local_coordinates);
      for (size_t component_index = 0;
           component_index < number_of_dofs_per_node; component_index++) {
        Eigen::VectorXd f_component(number_of_dofs_per_node);
        f_component(0) = nodal_force_vector(component_index);
        f_component(1) =
            nodal_force_vector(component_index + number_of_dofs_per_node);
        double interpolated_force_component =
            N(0, 0) * f_component(0) + N(0, 1) * f_component(1);
        element_rhs(component_index) +=
            N(0, 0) * interpolated_force_component * integration_point.weight();
        element_rhs(component_index + number_of_dofs_per_node) +=
            N(0, 1) * interpolated_force_component * integration_point.weight();
      }
    }

    // Scatter
    for (size_t i_node = 0; i_node < nodes.size(); i_node++) {
      auto number_of_dofs = nodes[i_node]->number_of_dofs();
      for (size_t i_dof = 0; i_dof < number_of_dofs; i_dof++) {
        auto local_row_index = i_node * number_of_dofs_per_node + i_dof;
        auto global_row_index =
            number_of_dofs_per_node * nodes[i_node]->id() + i_dof;

        global_rhs(global_row_index) += element_rhs(local_row_index);
      }
    }

    element_rhs.setZero();
  }
  std::cout << "Global rhs = " << global_rhs << std::endl;

  // Apply BCs by penalty
  double penalty = 1e8;
  double boundary_value =
      0.0;  // This should be changed to accept a lambda with the components

  for (const auto& element : mesh.dirichlet_boundary_) {
    for (const auto& node : element->nodes()) {
      auto node_id = node->id();
      for (size_t component_index = 0;
           component_index < number_of_dofs_per_node; component_index++) {
        auto dof_id = node_id * number_of_dofs_per_node + component_index;
        global_K.coeffRef(dof_id, dof_id) *= penalty;
        global_rhs(dof_id) = boundary_value * penalty;
      }
    }
  }

  std::cout << "Modified global_K\n" << global_K << std::endl;

  std::cout << "Modified global_rhs = " << global_rhs << std::endl;

  Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg_solver;
  cg_solver.compute(global_K);
  Eigen::VectorXd x(mesh.number_of_dofs());
  for (int i = 0; i < 100; i++) {
    x = cg_solver.solve(global_rhs);
  }
  std::cout << "#iterations:     " << cg_solver.iterations() << std::endl;
  std::cout << "estimated error: " << cg_solver.error() << std::endl;

  std::cout << "x = " << x << std::endl;

  for (int i = 3; i >= 0; i--) {
    for (int j = 0; j < 4; j++) {
      auto index = (i * 4 + j) * number_of_dofs_per_node + 1;
      std::cout << x(index) << "\t\t";
    }
    std::cout << std::endl;
  }

  return 0;
}