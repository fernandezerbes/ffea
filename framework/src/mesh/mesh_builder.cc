#include "../../inc/mesh/mesh_builder.h"

namespace ffea {

Mesh MeshBuilder::Build(size_t number_of_dofs_per_node) {
  Mesh mesh(number_of_dofs_per_node);
  AddNodes(mesh);
  AddElements(mesh);
  return mesh;
}

MeshFromFileBuilder::MeshFromFileBuilder(const std::string &file_path)
    : file_stream_(), parser_(), mesh_data_() {
  file_stream_.open(file_path);
}

MeshFromFileBuilder::~MeshFromFileBuilder() { file_stream_.close(); }

void MeshFromFileBuilder::AddNodes(Mesh &mesh) {
  parser_.Parse(file_stream_, mesh_data_);
}

void MeshFromFileBuilder::AddElements(Mesh &mesh) {}

CartesianMeshBuilder::CartesianMeshBuilder(double x_length, double y_length,
                                           size_t elements_in_x,
                                           size_t elements_in_y)
    : x_length_(x_length),
      y_length_(y_length),
      elements_in_x_(elements_in_x),
      elements_in_y_(elements_in_y),
      nodes_in_x_(elements_in_x_ + 1),
      nodes_in_y_(elements_in_y_ + 1) {}

CartesianMeshBuilder::~CartesianMeshBuilder() {}

void CartesianMeshBuilder::AddNodes(Mesh &mesh) {
  double dx = x_length_ / elements_in_x_;
  double dy = y_length_ / elements_in_y_;
  double origin_x = 0.0;
  double origin_y = 0.0;
  for (size_t j_node = 0; j_node < nodes_in_y_; j_node++) {
    for (size_t i_node = 0; i_node < nodes_in_x_; i_node++) {
      double x = origin_x + i_node * dx;
      double y = origin_y + j_node * dy;
      mesh.AddNode({x, y, 0.0});
    }
  }
}

void CartesianMeshBuilder::AddElements(Mesh &mesh) {
  std::string body_name = "surface";
  for (size_t j_element = 0; j_element < elements_in_y_; j_element++) {
    for (size_t i_element = 0; i_element < elements_in_x_; i_element++) {
      size_t index = j_element * elements_in_x_ + i_element;
      size_t first_node_index = index + j_element;
      size_t second_node_index = first_node_index + 1;
      size_t third_node_index =
          second_node_index + nodes_in_x_;  // counter-clockwise
      size_t fourth_node_index =
          first_node_index + nodes_in_x_;  // counter-clockwise
      mesh.AddElement(ffea::ElementType::kFourNodeQuad, body_name,
                      {first_node_index, second_node_index, third_node_index,
                       fourth_node_index});
    }
  }

  std::string bottom_edge = "bottom_edge";
  std::string top_edge = "top_edge";

  for (size_t i_element = 0; i_element < elements_in_x_; i_element++) {
    size_t index_bottom = i_element;
    size_t first_node_index_bottom = index_bottom;
    size_t second_node_index_bottom = first_node_index_bottom + 1;
    mesh.AddElement(ffea::ElementType::kTwoNodeLine, bottom_edge,
                    {first_node_index_bottom, second_node_index_bottom});

    size_t index_top = i_element + (nodes_in_x_ * (nodes_in_y_ - 1));
    size_t first_node_index_top = index_top;
    size_t second_node_index_top = first_node_index_top + 1;
    mesh.AddElement(ffea::ElementType::kTwoNodeLine, top_edge,
                    {first_node_index_top, second_node_index_top});
  }
}

}  // namespace ffea
