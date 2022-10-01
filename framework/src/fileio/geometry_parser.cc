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

NodeData::NodeData(size_t id, const std::array<double, 3> &coords)
    : id(id), coords(coords) {}

GeometricEntityData::GeometricEntityData(size_t id,
                                         size_t geometric_entity_type,
                                         const std::vector<size_t> &node_ids)
    : id(id), type(geometric_entity_type), node_ids(node_ids) {}

GeometricEntityDataGroup::GeometricEntityDataGroup(
    size_t id, const std::string &group_name)
    : id_(id), name_(group_name), geometric_entities_ids_() {}

void GeometricEntityDataGroup::RegisterGeometricEntity(
    size_t geometric_entity_id) {
  geometric_entities_ids_.push_back(geometric_entity_id);
}

size_t GeometricEntityDataGroup::id() const { return id_; }

const std::string &GeometricEntityDataGroup::name() const { return name_; }

const std::vector<size_t> &GeometricEntityDataGroup::geometric_entities_ids()
    const {
  return geometric_entities_ids_;
}

void GeometryData::AddNode(size_t id, const std::array<double, 3> &coords) {
  nodes_.emplace_back(id, coords);
}

void GeometryData::AddGeometricEntityDataGroup(size_t id,
                                               const std::string &group_name) {
  geometric_entities_groups_.emplace_back(id, group_name);
}

void GeometryData::AddGeometricEntityData(size_t id,
                                          size_t geometric_entity_type,
                                          const std::vector<size_t> &node_ids,
                                          size_t owner_shape_dimension,
                                          size_t owner_shape_id) {
  geometric_entities_.emplace_back(id, geometric_entity_type, node_ids);
  auto &map = shape_id_to_entities_group_ids_maps_[owner_shape_dimension];
  auto &entity_group_ids = map.at(owner_shape_id);
  for (auto group_id : entity_group_ids) {
    auto &geometric_entity_group = GetGeometricEntityDataGroup(group_id);
    geometric_entity_group.RegisterGeometricEntity(id);
  }
}

void GeometryData::RegisterShapeId(size_t dimensions, size_t shape_id,
                                   size_t entity_group_id) {
  auto &map = shape_id_to_entities_group_ids_maps_[dimensions];
  if (map.contains(shape_id)) {
    auto &entity_group_ids = map.at(shape_id);
    entity_group_ids.push_back(entity_group_id);
  } else {
    map.insert({shape_id, {entity_group_id}});
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
    size_t group_id) {
  for (auto &group : geometric_entities_groups_) {
    if (group.id() == group_id) {
      return group;
    }
  }

  throw std::runtime_error("GeometricEntityDataGroup with id = " +
                           std::to_string(group_id) + " not found");
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
  int id;
  std::string name;
  for (size_t group_idx = 0; group_idx < number_of_groups; group_idx++) {
    file >> dimensions >> id >> name;
    utilities::ConvertOneToZeroBased(id);
    name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
    geometry_data.AddGeometricEntityDataGroup(id, name);
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
      int shape_id;
      file >> shape_id;  // column = 0
      utilities::ConvertOneToZeroBased(shape_id);

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
        int entity_group_id;
        file >> entity_group_id;
        utilities::ConvertOneToZeroBased(entity_group_id);
        geometry_data.RegisterShapeId(dimensions, shape_id, entity_group_id);
      }

      if (dimensions > 0) {
        size_t number_of_bounding_shapes;
        file >> number_of_bounding_shapes;
        int bounding_shape_id;
        for (size_t bounding_shape_idx = 0;
             bounding_shape_idx < number_of_bounding_shapes;
             bounding_shape_idx++) {
          file >> bounding_shape_id;
        }
      }
    }
  }
}

void NodesParser::Parse(std::ifstream &file,
                        GeometryData &geometry_data) const {
  size_t number_of_entity_blocks;
  size_t number_of_nodes;
  size_t minimum_node_id;
  size_t maximum_node_id;
  file >> number_of_entity_blocks >> number_of_nodes >> minimum_node_id >>
      maximum_node_id;

  size_t owner_shape_dimension;
  int owner_shape_id;
  size_t parametric;
  size_t number_of_nodes_in_block;
  for (size_t entity_block_idx = 0; entity_block_idx < number_of_entity_blocks;
       entity_block_idx++) {
    file >> owner_shape_dimension >> owner_shape_id >> parametric >>
        number_of_nodes_in_block;

    size_t node_id;
    std::vector<size_t> node_ids;
    node_ids.reserve(number_of_nodes_in_block);
    for (size_t node_idx = 0; node_idx < number_of_nodes_in_block; node_idx++) {
      file >> node_id;
      utilities::ConvertOneToZeroBased(node_id);
      node_ids.push_back(node_id);
    }

    double x;
    double y;
    double z;
    for (size_t node_idx = 0; node_idx < number_of_nodes_in_block; node_idx++) {
      file >> x >> y >> z;
      geometry_data.AddNode(node_ids[node_idx], {x, y, z});
    }
  }
}

void GeometricEntitiesParser::Parse(std::ifstream &file,
                                    GeometryData &geometry_data) const {
  size_t number_of_entity_blocks;
  size_t number_of_entities;
  size_t minimum_entity_id;
  size_t maximum_entity_id;
  file >> number_of_entity_blocks >> number_of_entities >> minimum_entity_id >>
      maximum_entity_id;

  size_t total_number_of_entities = maximum_entity_id - minimum_entity_id + 1;
  if (number_of_entities != total_number_of_entities) {
    throw std::runtime_error("Partitioned meshes are not supported");
  }

  size_t owner_shape_dimension;
  int owner_shape_id;
  size_t geometric_entity_type;
  size_t number_of_entities_in_block;
  for (size_t entity_block_idx = 0; entity_block_idx < number_of_entity_blocks;
       entity_block_idx++) {
    file >> owner_shape_dimension >> owner_shape_id >> geometric_entity_type >>
        number_of_entities_in_block;
    utilities::ConvertOneToZeroBased(owner_shape_id);

    size_t node_id;
    size_t entity_id;
    std::vector<size_t> node_ids;
    std::string line;
    for (size_t entity_idx = 0; entity_idx < number_of_entities_in_block;
         entity_idx++) {
      node_ids.clear();
      std::getline(file, line);
      // TODO Make this better, don't know why getline is returning an empty
      // string
      if (line == "") {
        std::getline(file, line);
      }
      std::istringstream ss(line);
      ss >> entity_id;
      utilities::ConvertOneToZeroBased(entity_id);
      while (ss >> node_id) {
        utilities::ConvertOneToZeroBased(node_id);
        node_ids.push_back(node_id);
      }
      geometry_data.AddGeometricEntityData(entity_id, geometric_entity_type,
                                           node_ids, owner_shape_dimension,
                                           owner_shape_id);
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
