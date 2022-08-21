#include "../../inc/fileio/mesh_parser.h"

#include <algorithm>
#include <iostream>
#include <sstream>
namespace ffea {

NodeData::NodeData(size_t id, const std::array<double, 3> &coords)
    : id(id), coords(coords) {}

ElementData::ElementData(size_t element_type,
                         const std::vector<size_t> &node_ids)
    : type(element_type), node_ids(node_ids) {}

ElementGroup::ElementGroup(const std::string &group_name)
    : name(group_name), elements() {}

void ElementGroup::AddElement(size_t element_type,
                              const std::vector<size_t> &node_ids) {
  elements.emplace_back(element_type, node_ids);
}

void MeshData::AddNode(size_t id, const std::array<double, 3> &coords) {
  nodes.emplace_back(id, coords);
}

void MeshData::AddElementGroup(const std::string &group_name) {
  element_groups.emplace_back(group_name);
}

void MeshData::AddElement(size_t element_type, size_t element_group_id,
                          const std::vector<size_t> &node_ids) {
  element_groups[element_group_id].AddElement(element_type, node_ids);
}

void MeshParser::Parse(std::ifstream &file, MeshData &mesh_data) {
  std::string line;
  while (std::getline(file, line)) {
    auto parser = SectionParserFactory::CreateSectionParser(line);
    if (parser != nullptr) {
      parser->Parse(file, mesh_data);
    }
  }
}

void GroupNamesParser::Parse(std::ifstream &file, MeshData &mesh_data) {
  size_t number_of_groups;
  file >> number_of_groups;

  size_t dimension;
  size_t physical_tag;
  std::string name;
  for (size_t group_index = 0; group_index < number_of_groups; group_index++) {
    file >> dimension >> physical_tag >> name;
    name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
    mesh_data.AddElementGroup(name);
  }
}

void NodesParser::Parse(std::ifstream &file, MeshData &mesh_data) {
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

void ElementsParser::Parse(std::ifstream &file, MeshData &mesh_data) {
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
      mesh_data.AddElement(element_type, entity_block_index, node_ids);
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
    return std::make_unique<ElementsParser>();
  }
  return nullptr;
}

}  // namespace ffea
