#include "../../inc/fileio/geometry_parser.h"

#include <algorithm>
#include <iostream>
#include <sstream>
namespace ffea {

NodeData::NodeData(size_t id, const std::array<double, 3> &coords)
    : id(id), coords(coords) {}

GeometricEntityData::GeometricEntityData(size_t geometric_entity_type,
                                         const std::vector<size_t> &node_ids)
    : type(geometric_entity_type), node_ids(node_ids) {}

GeometricEntityDataGroup::GeometricEntityDataGroup(
    const std::string &group_name)
    : name(group_name), geometric_entities_() {}

void GeometricEntityDataGroup::AddGeometricEntity(
    size_t geometric_entity_type, const std::vector<size_t> &node_ids) {
  geometric_entities_.emplace_back(geometric_entity_type, node_ids);
}

void GeometryData::AddNode(size_t id, const std::array<double, 3> &coords) {
  nodes_.emplace_back(id, coords);
}

void GeometryData::AddGeometricEntityDataGroup(const std::string &group_name) {
  geometric_entities_groups_.emplace_back(group_name);
}

void GeometryData::AddGeometricEntityData(size_t geometric_entity_type,
                                          size_t geometric_entity_group_id,
                                          const std::vector<size_t> &node_ids) {
  geometric_entities_groups_[geometric_entity_group_id].AddGeometricEntity(
      geometric_entity_type, node_ids);
}

const std::vector<NodeData> &GeometryData::nodes() const { return nodes_; }

const std::vector<GeometricEntityDataGroup>
    &GeometryData::geometric_entities_groups() const {
  return geometric_entities_groups_;
}

void GeometryParser::Parse(std::ifstream &file, GeometryData &mesh_data) {
  std::string line;
  while (std::getline(file, line)) {
    auto parser = SectionParserFactory::CreateSectionParser(line);
    if (parser != nullptr) {
      parser->Parse(file, mesh_data);
    }
  }
}

void GroupNamesParser::Parse(std::ifstream &file, GeometryData &mesh_data) {
  size_t number_of_groups;
  file >> number_of_groups;

  size_t dimension;
  size_t physical_tag;
  std::string name;
  for (size_t group_index = 0; group_index < number_of_groups; group_index++) {
    file >> dimension >> physical_tag >> name;
    name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
    mesh_data.AddGeometricEntityDataGroup(name);
  }
}

void NodesParser::Parse(std::ifstream &file, GeometryData &mesh_data) {
  size_t number_of_entity_blocks;
  size_t number_of_nodes;
  size_t minimum_node_tag;
  size_t maximum_node_tag;
  file >> number_of_entity_blocks >> number_of_nodes >> minimum_node_tag >>
      maximum_node_tag;

  size_t entity_dimension;
  size_t entity_tag;
  size_t parametric;
  size_t number_of_nodes_in_block;
  for (size_t entity_block_index = 0;
       entity_block_index < number_of_entity_blocks; entity_block_index++) {
    file >> entity_dimension >> entity_tag >> parametric >>
        number_of_nodes_in_block;

    size_t node_tag;
    std::vector<size_t> node_ids;
    node_ids.reserve(number_of_nodes_in_block);
    for (size_t node_index = 0; node_index < number_of_nodes_in_block;
         node_index++) {
      file >> node_tag;
      node_ids.push_back(node_tag - 1);
    }

    double x;
    double y;
    double z;
    for (size_t node_index = 0; node_index < number_of_nodes_in_block;
         node_index++) {
      file >> x >> y >> z;
      mesh_data.AddNode(node_ids[node_index], {x, y, z});
    }
  }
}

void GeometricEntitiesParser::Parse(std::ifstream &file,
                                    GeometryData &mesh_data) {
  size_t number_of_entity_blocks;  // One entity block per physical group
  size_t number_of_elements;
  size_t minimum_element_tag;
  size_t maximum_element_tag;
  file >> number_of_entity_blocks >> number_of_elements >>
      minimum_element_tag >> maximum_element_tag;

  size_t entity_dimension;
  size_t entity_tag;  // entity_tag is the tag of the physical entity
  size_t element_type;
  size_t number_of_elements_in_block;
  for (size_t entity_block_index = 0;
       entity_block_index < number_of_entity_blocks; entity_block_index++) {
    file >> entity_dimension >> entity_tag >> element_type >>
        number_of_elements_in_block;

    size_t node_tag;
    size_t element_tag;
    std::vector<size_t> node_ids;
    std::string line;
    for (size_t element_index = 0; element_index < number_of_elements_in_block;
         element_index++) {
      node_ids.clear();
      std::getline(file, line);
      // TODO Make this better, don't know why getline is returning an empty
      // string
      if (line == "") {
        std::getline(file, line);
      }
      std::istringstream ss(line);
      ss >> element_tag;
      while (ss >> node_tag) {
        node_ids.push_back(node_tag - 1);  // Gmsh is 1-based, ffea is 0-based
      }
      mesh_data.AddGeometricEntityData(element_type, entity_block_index,
                                       node_ids);
    }
  }
}

std::unique_ptr<Parser> SectionParserFactory::CreateSectionParser(
    const std::string &section_name) {
  if (section_name == "$PhysicalNames") {
    return std::make_unique<GroupNamesParser>();
  } else if (section_name == "$Nodes") {
    return std::make_unique<NodesParser>();
  } else if (section_name == "$Elements") {
    return std::make_unique<GeometricEntitiesParser>();
  }
  return nullptr;
}

}  // namespace ffea
