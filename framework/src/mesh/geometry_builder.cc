#include "../../inc/mesh/geometry_builder.h"

namespace ffea {

GeometryBuilder::GeometryBuilder(const GeometricEntityFactory &factory)
    : geometric_entity_factory_(factory) {}

Geometry GeometryBuilder::Build() {
  Geometry geometry;
  AddNodes(geometry);
  AddEntities(geometry);
  return geometry;
}

GeometryFromFileBuilder::GeometryFromFileBuilder(
    const std::string &file_path, const GeometricEntityFactory &factory)
    : GeometryBuilder::GeometryBuilder(factory),
      file_stream_(),
      parser_(),
      geometry_data_() {
  file_stream_.open(file_path);
  parser_.Parse(file_stream_, geometry_data_);
}

GeometryFromFileBuilder::~GeometryFromFileBuilder() { file_stream_.close(); }

void GeometryFromFileBuilder::AddNodes(Geometry &geometry) {
  for (const auto &node : geometry_data_.nodes()) {
    geometry.AddNode(node.coords);
  }
}

void GeometryFromFileBuilder::AddEntities(Geometry &geometry) {
  for (const auto &geometric_entities_group :
       geometry_data_.geometric_entities_groups()) {
    for (const auto &geometric_entity :
         geometric_entities_group.geometric_entities()) {
      // TODO Use mapping for gmsh elements instead of doing
      // ElementType(element.type
      // - 1)
      geometry.AddGeometricEntity(
          GeometricEntityType(geometric_entity.type - 1),
          geometric_entities_group.name(), geometric_entity.node_ids,
          geometric_entity_factory_);
    }
  }
}

// CartesianGeometryBuilder::CartesianGeometryBuilder(double x_length, double
// y_length,
//                                            size_t elements_in_x,
//                                            size_t elements_in_y)
//     : x_length_(x_length),
//       y_length_(y_length),
//       elements_in_x_(elements_in_x),
//       elements_in_y_(elements_in_y),
//       nodes_in_x_(elements_in_x_ + 1),
//       nodes_in_y_(elements_in_y_ + 1) {}

// CartesianGeometryBuilder::~CartesianGeometryBuilder() {}

// void CartesianGeometryBuilder::AddNodes(Geometry &geometry) {
//   double dx = x_length_ / elements_in_x_;
//   double dy = y_length_ / elements_in_y_;
//   double origin_x = 0.0;
//   double origin_y = 0.0;
//   for (size_t j_node = 0; j_node < nodes_in_y_; j_node++) {
//     for (size_t i_node = 0; i_node < nodes_in_x_; i_node++) {
//       double x = origin_x + i_node * dx;
//       double y = origin_y + j_node * dy;
//       geometry.AddNode({x, y, 0.0});
//     }
//   }
// }

// void CartesianGeometryBuilder::AddEntities(Geometry &geometry) {
//   std::string body_name = "surface";
//   for (size_t j_element = 0; j_element < elements_in_y_; j_element++) {
//     for (size_t i_element = 0; i_element < elements_in_x_; i_element++) {
//       size_t index = j_element * elements_in_x_ + i_element;
//       size_t first_node_index = index + j_element;
//       size_t second_node_index = first_node_index + 1;
//       size_t third_node_index =
//           second_node_index + nodes_in_x_;  // counter-clockwise
//       size_t fourth_node_index =
//           first_node_index + nodes_in_x_;  // counter-clockwise
//       geometry.AddElement(ffea::GeometricEntityType::kFourNodeQuad,
//       body_name,
//                       {first_node_index, second_node_index, third_node_index,
//                        fourth_node_index});
//     }
//   }

//   std::string bottom_edge = "dirichlet";
//   std::string top_edge = "neumann";

//   for (size_t i_element = 0; i_element < elements_in_x_; i_element++) {
//     size_t index_bottom = i_element;
//     size_t first_node_index_bottom = index_bottom;
//     size_t second_node_index_bottom = first_node_index_bottom + 1;
//     geometry.AddElement(ffea::GeometricEntityType::kTwoNodeLine, bottom_edge,
//                     {first_node_index_bottom, second_node_index_bottom});

//     size_t index_top = i_element + (nodes_in_x_ * (nodes_in_y_ - 1));
//     size_t first_node_index_top = index_top;
//     size_t second_node_index_top = first_node_index_top + 1;
//     geometry.AddElement(ffea::GeometricEntityType::kTwoNodeLine, top_edge,
//                     {first_node_index_top, second_node_index_top});
//   }
// }

}  // namespace ffea
