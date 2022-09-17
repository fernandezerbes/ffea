#ifndef FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_
#define FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_

#include <array>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ffea {

struct NodeData {
  NodeData(size_t id, const std::array<double, 3> &coords);
  const size_t id;
  const std::array<double, 3> coords;
};

struct GeometricEntityData {
  GeometricEntityData(size_t geometric_entity_type,
                      const std::vector<size_t> &node_ids);
  const size_t type;
  const std::vector<size_t> node_ids;
};

class GeometricEntityDataGroup {
 public:
  GeometricEntityDataGroup(const std::string &group_name);
  void AddGeometricEntity(size_t geometric_entity_type,
                          const std::vector<size_t> &node_ids);
  const std::string &name() const;
  const std::vector<GeometricEntityData> &geometric_entities() const;

 private:
  const std::string name_;
  std::vector<GeometricEntityData> geometric_entities_;
};

class GeometryData {
 public:
  void AddNode(size_t id, const std::array<double, 3> &coords);
  void AddGeometricEntityDataGroup(const std::string &group_name);
  void AddGeometricEntityData(size_t geometric_entity_type,
                              size_t geometric_entity_group_id,
                              const std::vector<size_t> &node_ids);
  void RegisterShapeTag(size_t dimension, size_t shape_tag,
                        size_t entity_group_tag);
  const std::vector<NodeData> &nodes() const;
  const std::vector<GeometricEntityDataGroup> &geometric_entities_groups()
      const;

 private:
  std::vector<NodeData> nodes_;
  std::vector<GeometricEntityDataGroup> geometric_entities_groups_;
  std::array<std::multimap<size_t, size_t>, 4>
      shape_tag_to_entities_group_tags_maps_;
};

class Parser {
 public:
  virtual void Parse(std::ifstream &file,
                     GeometryData &geometry_data) const = 0;
};

class GeometryParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file,
                     GeometryData &geometry_data) const override;
};

class GroupNamesParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file,
                     GeometryData &geometry_data) const override;
};

class NodesParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file,
                     GeometryData &geometry_data) const override;
};

class GeometricEntitiesParser : public Parser {
 public:
  virtual void Parse(std::ifstream &file,
                     GeometryData &geometry_data) const override;
};

class SectionParserFactory {
 public:
  static std::unique_ptr<Parser> CreateSectionParser(
      const std::string &section_name);
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_
