#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include "../framework/inc/math/CustomDenseMatrix.h"
#include "../framework/inc/mesh/Coordinates.h"
#include "../framework/inc/mesh/DegreeOfFreedom.h"
#include "../framework/inc/mesh/IntegrationPoint.h"
#include "../framework/inc/mesh/Node.h"
#include "../framework/inc/mesh/Line2.h"
#include "../framework/inc/mesh/Quad4.h"
#include "../framework/inc/mesh/Mesh.h"

#include "../framework/inc/math/Utils.h"

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
  body.push_back(new ffea::Quad4({&nodes[5], &nodes[6], &nodes[10], &nodes[9]}));
  body.push_back(new ffea::Quad4({&nodes[6], &nodes[7], &nodes[11], &nodes[10]}));
  body.push_back(new ffea::Quad4({&nodes[8], &nodes[9], &nodes[13], &nodes[12]}));
  body.push_back(new ffea::Quad4({&nodes[9], &nodes[10], &nodes[14], &nodes[13]}));
  body.push_back(new ffea::Quad4({&nodes[10], &nodes[11], &nodes[15], &nodes[14]}));

  std::vector<ffea::IntegrationPoint> quad_integration_points;
  quad_integration_points.push_back(ffea::IntegrationPoint(
    {-0.5773502691896257, -0.5773502691896257, 0.0},1.0));
  quad_integration_points.push_back(ffea::IntegrationPoint(
    {0.5773502691896257, -0.5773502691896257, 0.0}, 1.0));
  quad_integration_points.push_back(ffea::IntegrationPoint(
    {-0.5773502691896257, 0.5773502691896257, 0.0}, 1.0));
  quad_integration_points.push_back(ffea::IntegrationPoint(
    {0.5773502691896257, 0.5773502691896257, 0.0}, 1.0));

  std::vector<ffea::IntegrationPoint> line_integration_points;
  line_integration_points.push_back(ffea::IntegrationPoint(
    {-0.5773502691896257, 0.0, 0.0},1.0));
  line_integration_points.push_back(ffea::IntegrationPoint(
    {0.5773502691896257, 0.0, 0.0}, 1.0));

  ffea::Mesh mesh(number_of_dofs_per_node, nodes);
  // mesh.RegisterElementGroup("dirichlet_boundary", dirichlet_boundary);
  // mesh.RegisterElementGroup("neumann_boundary", neumann_boundary);
  // mesh.RegisterElementGroup("body", body);
  mesh.dirichlet_boundary_ = dirichlet_boundary;
  mesh.neumann_boundary_ = neumann_boundary;
  mesh.body_ = body;

  double nu = 0.3;
  double E = 210e9;
  double factor = E / (1 - nu * nu);
  Eigen::MatrixXd C(3, 3);
  C(0, 0) = factor;
  C(0, 1) = factor * nu;
  C(1, 0) = C(0, 1);
  C(1, 1) = factor;
  C(2, 2) = (1 - nu) * factor;

  auto number_of_dofs = mesh.number_of_dofs();
  Eigen::SparseMatrix<double, Eigen::RowMajor> global_K(
    number_of_dofs, number_of_dofs);
  std::vector<Eigen::Triplet<double>> coefficients;
  Eigen::VectorXd global_rhs(number_of_dofs);

  // const auto& elements = mesh.GetElementGroup("body");

  std::cout << global_K << std::endl;

  for (const auto& element: mesh.body_) {
    size_t number_of_nodes = element->number_of_nodes();
    size_t number_of_dofs = element->number_of_dofs();
    Eigen::MatrixXd element_K(number_of_dofs, number_of_dofs);
    const auto& element_nodes_coordinates = element->node_coordinates();
    for (const auto& integration_point: quad_integration_points) {
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
      element_K += B.transpose() * C * B * jacobian.determinant();
    }

    // Scatter coefficients
    auto nodes = element->nodes();
    for (size_t i_node = 0; i_node < nodes.size(); i_node++) {
      auto number_of_dofs = nodes[i_node]->number_of_dofs();
      for (size_t j_node = 0; j_node < nodes.size(); j_node++) {
        for (size_t i_dof = 0; i_dof < number_of_dofs; i_dof++) {
          for (size_t j_dof = 0; j_dof < number_of_dofs; j_dof++) {
            auto local_row_index = i_node + i_dof;
            auto local_col_index = j_node + j_dof;
            auto global_row_index = number_of_dofs_per_node * nodes[i_node]->id() + i_dof;
            auto global_col_index = number_of_dofs_per_node * nodes[j_node]->id() + j_dof;

            coefficients.push_back(Eigen::Triplet<double>(
              global_row_index, global_col_index,
              element_K(local_row_index, local_col_index)));
          }
        }
      }
    }
  }

  global_K.setFromTriplets(coefficients.begin(), coefficients.end());
  global_K.makeCompressed();

  std::cout << global_K << std::endl;
  
  return 0;
}