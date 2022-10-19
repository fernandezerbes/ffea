#include "../../inc/geometry/geometry_builder.h"

namespace ffea {

GeometryBuilder::GeometryBuilder(const GeometricEntityFactory &factory)
    : factory_(factory) {}

Geometry GeometryBuilder::Build() {
  Geometry geometry;
  AddNodes(geometry);
  AddEntities(geometry);
  AddEntitiesGroups(geometry);
  return geometry;
}

GeometryFromFileBuilder::GeometryFromFileBuilder(
    const std::string &file_path, const GeometricEntityFactory &factory)
    : GeometryBuilder::GeometryBuilder(factory),
      file_stream_(),
      parser_(),
      data_() {
  file_stream_.open(file_path);
  parser_.Parse(file_stream_, data_);
  file_stream_.close();
}

void GeometryFromFileBuilder::AddNodes(Geometry &geometry) {
  for (const auto &node : data_.nodes()) {
    geometry.AddNode(node.coords);
  }
}

void GeometryFromFileBuilder::AddEntities(Geometry &geometry) {
  for (const auto &entity : data_.geometric_entities()) {
    // TODO Use mapping for gmsh elements instead of doing
    // ElementType(element.type
    // - 1)
    geometry.AddGeometricEntity(GeometricEntityType(entity.type - 1),
                                entity.node_tags, factory_);
  }
}

void GeometryFromFileBuilder::AddEntitiesGroups(Geometry &geometry) {
  for (const auto &entity_group : data_.geometric_entity_groups()) {
    geometry.RegisterGeometricEntityGroup(entity_group.name(),
                                          entity_group.entity_tags());
  }
}

}  // namespace ffea
