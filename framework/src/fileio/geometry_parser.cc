#include "../../inc/fileio/geometry_parser.h"

#include <math.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace ffea {

namespace utilities {

void ConvertOneToZeroBased(size_t &idx) { idx--; }
void ConvertOneToZeroBased(int &idx) { idx--; }
void ConvertZeroToOneBased(size_t &idx) { idx++; }
void ConvertZeroToOneBased(int &idx) { idx++; }

}  // namespace utilities

NodeData::NodeData(size_t tag, const std::array<double, 3> &coords) : tag(tag), coords(coords) {}

GeometricEntityData::GeometricEntityData(size_t tag, size_t type,
                                         const std::vector<size_t> &node_tags)
    : tag(tag), type(type), node_tags(node_tags) {}

GeometricEntityDataGroup::GeometricEntityDataGroup(size_t tag, const std::string &name)
    : tag_(tag), name_(name) {}

size_t GeometricEntityDataGroup::tag() const { return tag_; }

const std::string &GeometricEntityDataGroup::name() const { return name_; }

const std::vector<size_t> &GeometricEntityDataGroup::entity_tags() const { return entity_tags_; }

void GeometricEntityDataGroup::RegisterGeometricEntity(size_t tag) { entity_tags_.push_back(tag); }

const std::vector<NodeData> &GeometryData::nodes() const { return nodes_; }

const std::vector<GeometricEntityData> &GeometryData::geometric_entities() const {
  return entities_;
}

const std::vector<GeometricEntityDataGroup> &GeometryData::geometric_entity_groups() const {
  return entity_groups_;
}

void GeometryData::AddNode(size_t tag, const std::array<double, 3> &coords) {
  nodes_.emplace_back(tag, coords);
}

void GeometryData::AddGeometricEntityData(size_t tag, size_t type,
                                          const std::vector<size_t> &node_tags,
                                          size_t owner_shape_dim, size_t owner_shape_tag) {
  entities_.emplace_back(tag, type, node_tags);
  auto &map = shape_tag_to_entity_group_tags_maps_[owner_shape_dim];
  auto &entity_group_tags = map.at(owner_shape_tag);
  for (auto entity_group_tag : entity_group_tags) {
    auto &group = GetGeometricEntityDataGroup(entity_group_tag);
    group.RegisterGeometricEntity(tag);
  }
}

void GeometryData::AddGeometricEntityDataGroup(size_t tag, const std::string &name) {
  entity_groups_.emplace_back(tag, name);
}

void GeometryData::RegisterShapeTag(size_t dim, size_t shape_tag, size_t entity_group_tag) {
  auto &map = shape_tag_to_entity_group_tags_maps_[dim];
  if (map.contains(shape_tag)) {
    auto &entity_group_tags = map.at(shape_tag);
    entity_group_tags.push_back(entity_group_tag);
  } else {
    map.insert({shape_tag, {entity_group_tag}});
  }
}

GeometricEntityDataGroup &GeometryData::GetGeometricEntityDataGroup(size_t group_tag) {
  for (auto &group : entity_groups_) {
    if (group.tag() == group_tag) {
      return group;
    }
  }

  throw std::runtime_error("GeometricEntityDataGroup with tag = " + std::to_string(group_tag) +
                           " not found");
}

void GeometryParser::Parse(std::ifstream &file, GeometryData &data) const {
  std::string line;
  while (std::getline(file, line)) {
    auto parser = SectionParserFactory::CreateSectionParser(line);
    if (parser != nullptr) {
      parser->Parse(file, data);
    }
  }
}

void GroupNamesParser::Parse(std::ifstream &file, GeometryData &data) const {
  size_t number_of_groups = 0;
  file >> number_of_groups;

  size_t dim = 0;
  int tag = 0;
  std::string name;
  for (size_t group_idx = 0; group_idx < number_of_groups; group_idx++) {
    file >> dim >> tag >> name;
    utilities::ConvertOneToZeroBased(tag);
    name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
    data.AddGeometricEntityDataGroup(tag, name);
  }
}

void ShapesParser::Parse(std::ifstream &file, GeometryData &data) const {
  std::array<size_t, 4> shapes_per_dim{};
  for (size_t dim = 0; dim < shapes_per_dim.size(); dim++) {
    file >> shapes_per_dim[dim];
  }

  for (size_t dim = 0; dim < shapes_per_dim.size(); dim++) {
    for (size_t shape_idx = 0; shape_idx < shapes_per_dim[dim]; shape_idx++) {
      int shape_tag = 0;
      file >> shape_tag;  // column = 0
      utilities::ConvertOneToZeroBased(shape_tag);

      const size_t end_coords_column = (dim == 0) ? 4 : 7;
      double coords = NAN;
      for (size_t column = 1; column < end_coords_column; column++) {
        file >> coords;
      }

      size_t number_of_entity_groups = 0;
      file >> number_of_entity_groups;
      for (size_t column = end_coords_column; column < end_coords_column + number_of_entity_groups;
           column++) {
        int entity_group_tag = 0;
        file >> entity_group_tag;
        utilities::ConvertOneToZeroBased(entity_group_tag);
        data.RegisterShapeTag(dim, shape_tag, entity_group_tag);
      }

      if (dim > 0) {
        size_t number_of_bounding_shapes = 0;
        file >> number_of_bounding_shapes;
        int bounding_shape_tag = 0;
        for (size_t bounding_shape_idx = 0; bounding_shape_idx < number_of_bounding_shapes;
             bounding_shape_idx++) {
          file >> bounding_shape_tag;
        }
      }
    }
  }
}

void NodesParser::Parse(std::ifstream &file, GeometryData &data) const {
  size_t number_of_entity_blocks = 0;
  size_t number_of_nodes = 0;
  size_t min_node_tag = 0;
  size_t max_node_tag = 0;
  file >> number_of_entity_blocks >> number_of_nodes >> min_node_tag >> max_node_tag;

  size_t owner_shape_dim = 0;
  int owner_shape_tag = 0;
  size_t parametric = 0;
  size_t number_of_nodes_in_block = 0;
  for (size_t block_idx = 0; block_idx < number_of_entity_blocks; block_idx++) {
    file >> owner_shape_dim >> owner_shape_tag >> parametric >> number_of_nodes_in_block;

    size_t node_tag = 0;
    std::vector<size_t> node_tags;
    node_tags.reserve(number_of_nodes_in_block);
    for (size_t node_idx = 0; node_idx < number_of_nodes_in_block; node_idx++) {
      file >> node_tag;
      utilities::ConvertOneToZeroBased(node_tag);
      node_tags.push_back(node_tag);
    }

    double x = NAN;
    double y = NAN;
    double z = NAN;
    for (size_t node_idx = 0; node_idx < number_of_nodes_in_block; node_idx++) {
      file >> x >> y >> z;
      data.AddNode(node_tags[node_idx], {x, y, z});
    }
  }
}

void GeometricEntitiesParser::Parse(std::ifstream &file, GeometryData &data) const {
  size_t number_of_entity_blocks = 0;
  size_t number_of_entities = 0;
  size_t min_entity_tag = 0;
  size_t max_entity_tag = 0;
  file >> number_of_entity_blocks >> number_of_entities >> min_entity_tag >> max_entity_tag;

  const size_t total_number_of_entities = max_entity_tag - min_entity_tag + 1;
  if (number_of_entities != total_number_of_entities) {
    throw std::runtime_error("Partitioned meshes are not supported");
  }

  size_t owner_shape_dim = 0;
  int owner_shape_tag = 0;
  size_t entity_type = 0;
  size_t number_of_entities_in_block = 0;
  for (size_t block_idx = 0; block_idx < number_of_entity_blocks; block_idx++) {
    file >> owner_shape_dim >> owner_shape_tag >> entity_type >> number_of_entities_in_block;
    utilities::ConvertOneToZeroBased(owner_shape_tag);

    size_t node_tag = 0;
    size_t entity_tag = 0;
    std::vector<size_t> node_tags;
    std::string line;
    for (size_t entity_idx = 0; entity_idx < number_of_entities_in_block; entity_idx++) {
      node_tags.clear();
      std::getline(file, line);
      // TODO Make this better, don't know why getline is returning an empty
      // string
      if (line.empty()) {
        std::getline(file, line);
      }
      std::istringstream ss(line);
      ss >> entity_tag;
      utilities::ConvertOneToZeroBased(entity_tag);
      while (ss >> node_tag) {
        utilities::ConvertOneToZeroBased(node_tag);
        node_tags.push_back(node_tag);
      }
      data.AddGeometricEntityData(entity_tag, entity_type, node_tags, owner_shape_dim,
                                  owner_shape_tag);
    }
  }
}

std::unique_ptr<Parser> SectionParserFactory::CreateSectionParser(const std::string &section_name) {
  if (section_name == "$PhysicalNames") {
    return std::make_unique<GroupNamesParser>();
  }

  if (section_name == "$Entities") {
    return std::make_unique<ShapesParser>();
  }

  if (section_name == "$Nodes") {
    return std::make_unique<NodesParser>();
  }

  if (section_name == "$Elements") {
    return std::make_unique<GeometricEntitiesParser>();
  }

  return nullptr;
}

}  // namespace ffea
