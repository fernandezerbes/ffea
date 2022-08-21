#include "../../inc/fileio/mesh_parser.h"

#include <iostream>

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
    std::cout << line << std::endl;
  }
}

std::string MeshParser::GetSectionName() {}

}  // namespace ffea
