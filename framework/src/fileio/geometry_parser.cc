#include "../../inc/fileio/geometry_parser.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace ffea {

namespace utilities {

void ConvertOneToZeroBased(size_t &index) { index--; }
void ConvertOneToZeroBased(int &index) { index--; }
void ConvertZeroToOneBased(size_t &index) { index++; }
void ConvertZeroToOneBased(int &index) { index++; }

}  // namespace utilities

NodeData::NodeData(size_t tag, const std::array<double, 3> &coords)
    : tag(tag), coords(coords) {}

GeometricEntityData::GeometricEntityData(size_t tag,
                                         size_t geometric_entity_type,
                                         const std::vector<size_t> &node_tags)
    : tag(tag), type(geometric_entity_type), node_tags(node_tags) {}

GeometricEntityDataGroup::GeometricEntityDataGroup(
    size_t tag, const std::string &group_name)
    : tag_(tag), name_(group_name), geometric_entity_tags_() {}

void GeometricEntityDataGroup::RegisterGeometricEntity(
    size_t geometric_entity_tag) {
  geometric_entity_tags_.push_back(geometric_entity_tag);
}

size_t GeometricEntityDataGroup::tag() const { return tag_; }

const std::string &GeometricEntityDataGroup::name() const { return name_; }

const std::vector<size_t> &GeometricEntityDataGroup::geometric_entity_tags()
    const {
  return geometric_entity_tags_;
}

void GeometryData::AddNode(size_t tag, const std::array<double, 3> &coords) {
  nodes_.emplace_back(tag, coords);
}

void GeometryData::AddGeometricEntityDataGroup(size_t tag,
                                               const std::string &group_name) {
  geometric_entities_groups_.emplace_back(tag, group_name);
}

void GeometryData::AddGeometricEntityData(size_t tag,
                                          size_t geometric_entity_type,
                                          const std::vector<size_t> &node_tags,
                                          size_t owner_shape_dimension,
                                          size_t owner_shape_tag) {
  geometric_entities_.emplace_back(tag, geometric_entity_type, node_tags);
  auto &map = shape_tag_to_entities_group_tags_maps_[owner_shape_dimension];
  auto &entity_group_tags = map.at(owner_shape_tag);
  for (auto group_tag : entity_group_tags) {
    auto &geometric_entity_group = GetGeometricEntityDataGroup(group_tag);
    geometric_entity_group.RegisterGeometricEntity(tag);
  }
}

void GeometryData::RegisterShapeTag(size_t dimensions, size_t shape_tag,
                                    size_t entity_group_tag) {
  auto &map = shape_tag_to_entities_group_tags_maps_[dimensions];
  if (map.contains(shape_tag)) {
    auto &entity_group_tags = map.at(shape_tag);
    entity_group_tags.push_back(entity_group_tag);
  } else {
    map.insert({shape_tag, {entity_group_tag}});
  }
}

const std::vector<NodeData> &GeometryData::nodes() const { return nodes_; }

const std::vector<GeometricEntityData> &GeometryData::geometric_entities()
    const {
  return geometric_entities_;
}

const std::vector<GeometricEntityDataGroup>
    &GeometryData::geometric_entities_groups() const {
  return geometric_entities_groups_;
}

GeometricEntityDataGroup &GeometryData::GetGeometricEntityDataGroup(
    size_t group_tag) {
  for (auto &group : geometric_entities_groups_) {
    if (group.tag() == group_tag) {
      return group;
    }
  }

  throw std::runtime_error("GeometricEntityDataGroup with tag = " +
                           std::to_string(group_tag) + " not found");
}

void GeometryParser::Parse(std::ifstream &file,
                           GeometryData &geometry_data) const {
  std::string line;
  while (std::getline(file, line)) {
    auto parser = SectionParserFactory::CreateSectionParser(line);
    if (parser != nullptr) {
      parser->Parse(file, geometry_data);
    }
  }
}

void GroupNamesParser::Parse(std::ifstream &file,
                             GeometryData &geometry_data) const {
  size_t number_of_groups;
  file >> number_of_groups;

  size_t dimensions;
  int tag;
  std::string name;
  for (size_t group_idx = 0; group_idx < number_of_groups; group_idx++) {
    file >> dimensions >> tag >> name;
    utilities::ConvertOneToZeroBased(tag);
    name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
    geometry_data.AddGeometricEntityDataGroup(tag, name);
  }
}

void ShapesParser::Parse(std::ifstream &file,
                         GeometryData &geometry_data) const {
  std::array<size_t, 4> shapes_per_dimension;
  for (size_t dimensions = 0; dimensions < shapes_per_dimension.size();
       dimensions++) {
    file >> shapes_per_dimension[dimensions];
  }

  for (size_t dimensions = 0; dimensions < shapes_per_dimension.size();
       dimensions++) {
    for (size_t shape_idx = 0; shape_idx < shapes_per_dimension[dimensions];
         shape_idx++) {
      int shape_tag;
      file >> shape_tag;  // column = 0
      utilities::ConvertOneToZeroBased(shape_tag);

      size_t end_coords_column = (dimensions == 0) ? 4 : 7;
      double coords;
      for (size_t column = 1; column < end_coords_column; column++) {
        file >> coords;
      }

      size_t number_of_geometric_entities_groups;
      file >> number_of_geometric_entities_groups;
      for (size_t column = end_coords_column;
           column < end_coords_column + number_of_geometric_entities_groups;
           column++) {
        int entity_group_tag;
        file >> entity_group_tag;
        utilities::ConvertOneToZeroBased(entity_group_tag);
        geometry_data.RegisterShapeTag(dimensions, shape_tag, entity_group_tag);
      }

      if (dimensions > 0) {
        size_t number_of_bounding_shapes;
        file >> number_of_bounding_shapes;
        int bounding_shape_tag;
        for (size_t bounding_shape_idx = 0;
             bounding_shape_idx < number_of_bounding_shapes;
             bounding_shape_idx++) {
          file >> bounding_shape_tag;
        }
      }
    }
  }
}

void NodesParser::Parse(std::ifstream &file,
                        GeometryData &geometry_data) const {
  size_t number_of_entity_blocks;
  size_t number_of_nodes;
  size_t minimum_node_tag;
  size_t maximum_node_tag;
  file >> number_of_entity_blocks >> number_of_nodes >> minimum_node_tag >>
      maximum_node_tag;

  size_t owner_shape_dimension;
  int owner_shape_tag;
  size_t parametric;
  size_t number_of_nodes_in_block;
  for (size_t entity_block_idx = 0; entity_block_idx < number_of_entity_blocks;
       entity_block_idx++) {
    file >> owner_shape_dimension >> owner_shape_tag >> parametric >>
        number_of_nodes_in_block;

    size_t node_tag;
    std::vector<size_t> node_tags;
    node_tags.reserve(number_of_nodes_in_block);
    for (size_t node_idx = 0; node_idx < number_of_nodes_in_block; node_idx++) {
      file >> node_tag;
      utilities::ConvertOneToZeroBased(node_tag);
      node_tags.push_back(node_tag);
    }

    double x;
    double y;
    double z;
    for (size_t node_idx = 0; node_idx < number_of_nodes_in_block; node_idx++) {
      file >> x >> y >> z;
      geometry_data.AddNode(node_tags[node_idx], {x, y, z});
    }
  }
}

void GeometricEntitiesParser::Parse(std::ifstream &file,
                                    GeometryData &geometry_data) const {
  size_t number_of_entity_blocks;
  size_t number_of_entities;
  size_t minimum_entity_tag;
  size_t maximum_entity_tag;
  file >> number_of_entity_blocks >> number_of_entities >> minimum_entity_tag >>
      maximum_entity_tag;

  size_t total_number_of_entities = maximum_entity_tag - minimum_entity_tag + 1;
  if (number_of_entities != total_number_of_entities) {
    throw std::runtime_error("Partitioned meshes are not supported");
  }

  size_t owner_shape_dimension;
  int owner_shape_tag;
  size_t geometric_entity_type;
  size_t number_of_entities_in_block;
  for (size_t entity_block_idx = 0; entity_block_idx < number_of_entity_blocks;
       entity_block_idx++) {
    file >> owner_shape_dimension >> owner_shape_tag >> geometric_entity_type >>
        number_of_entities_in_block;
    utilities::ConvertOneToZeroBased(owner_shape_tag);

    size_t node_tag;
    size_t entity_tag;
    std::vector<size_t> node_tags;
    std::string line;
    for (size_t entity_idx = 0; entity_idx < number_of_entities_in_block;
         entity_idx++) {
      node_tags.clear();
      std::getline(file, line);
      // TODO Make this better, don't know why getline is returning an empty
      // string
      if (line == "") {
        std::getline(file, line);
      }
      std::istringstream ss(line);
      ss >> entity_tag;
      utilities::ConvertOneToZeroBased(entity_tag);
      while (ss >> node_tag) {
        utilities::ConvertOneToZeroBased(node_tag);
        node_tags.push_back(node_tag);
      }
      geometry_data.AddGeometricEntityData(entity_tag, geometric_entity_type,
                                           node_tags, owner_shape_dimension,
                                           owner_shape_tag);
    }
  }
}

std::unique_ptr<Parser> SectionParserFactory::CreateSectionParser(
    const std::string &section_name) {
  if (section_name == "$PhysicalNames") {
    return std::make_unique<GroupNamesParser>();
  } else if (section_name == "$Entities") {
    return std::make_unique<ShapesParser>();
  } else if (section_name == "$Nodes") {
    return std::make_unique<NodesParser>();
  } else if (section_name == "$Elements") {
    return std::make_unique<GeometricEntitiesParser>();
  }
  return nullptr;
}

}  // namespace ffea
