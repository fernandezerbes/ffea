#include "../../inc/fileio/mesh_parser.h"

#include <iostream>
#include <stdexcept>

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
  while (!file.eof()) {
    file >> line;
    auto parser = SectionParserFactory::CreateSectionParser(line);
    parser->Parse(file, mesh_data);
    std::cout << line << std::endl;
  }
}

std::string MeshParser::GetSectionName() {}

void GroupNamesParser::Parse(std::ifstream &file, MeshData &mesh_data) {}

std::string GroupNamesParser::GetSectionName() {}

void NodesParser::Parse(std::ifstream &file, MeshData &mesh_data) {}

std::string NodesParser::GetSectionName() {}

void ElementsParser::Parse(std::ifstream &file, MeshData &mesh_data) {}

std::string ElementsParser::GetSectionName() {}

std::unique_ptr<Parser> SectionParserFactory::CreateSectionParser(
    const std::string &section_name) {
  if (section_name == "$PhysicalNames") {
    return std::make_unique<GroupNamesParser>();
  } else if (section_name == "$Nodes") {
    return std::make_unique<NodesParser>();
  } else if (section_name == "$Elements") {
    return std::make_unique<ElementsParser>();
  } else {
    throw std::logic_error("Section " + section_name +
                           " not implementd in parser.");
  }
}

}  // namespace ffea