#ifndef FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_
#define FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ffea {
// TODO Check constness of these member functions on all classes
struct NodeData {
  NodeData(size_t id, const std::array<double, 3> &coords);
  size_t id;
  std::array<double, 3> coords;
};

struct ElementData {
  ElementData(size_t element_type, const std::vector<size_t> &node_ids);
  size_t type;
  std::vector<size_t> node_ids;
};

struct ElementGroup {
  ElementGroup(const std::string &group_name);
  void AddElement(size_t element_type, const std::vector<size_t> &node_ids);
  std::string name;
  std::vector<ElementData> elements;
};

class MeshData {
 public:
  void AddNode(size_t id, const std::array<double, 3> &coords);
  void AddElementGroup(const std::string &group_name);
  void AddElement(size_t element_type, size_t element_group_id,
                  const std::vector<size_t> &node_ids);

  // TODO Make private
  //  private:
  std::vector<NodeData> nodes;
  std::vector<ElementGroup> element_groups;
};

class Parser {
 public:
  virtual void Parse(std::ifstream &file, MeshData &mesh_data) = 0;
};

class MeshParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file, MeshData &mesh_data) override;
};

class GroupNamesParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file, MeshData &mesh_data) override;
};

class NodesParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file, MeshData &mesh_data) override;
};

class ElementsParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file, MeshData &mesh_data) override;
};

class SectionParserFactory {
 public:
  static std::unique_ptr<Parser> CreateSectionParser(
      const std::string &section_name);
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_
