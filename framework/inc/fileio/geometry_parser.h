#ifndef FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_
#define FFEA_FRAMEWORK_INC_FILEIO_MESHPARSER_H_

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ffea {

namespace utilities {

void ConvertOneToZeroBased(size_t &index);
void ConvertOneToZeroBased(int &index);
void ConvertZeroToOneBased(size_t &index);
void ConvertZeroToOneBased(int &index);

}  // namespace utilities

struct NodeData {
  NodeData(size_t id, const std::array<double, 3> &coords);
  const size_t id;
  const std::array<double, 3> coords;
};

struct GeometricEntityData {
  GeometricEntityData(size_t id, size_t geometric_entity_type,
                      const std::vector<size_t> &node_ids);
  const size_t id;
  const size_t type;
  const std::vector<size_t> node_ids;
};

class GeometricEntityDataGroup {
 public:
  GeometricEntityDataGroup(size_t id, const std::string &group_name);
  void RegisterGeometricEntity(size_t geometric_entity_id);
  size_t id() const;
  const std::string &name() const;
  const std::vector<size_t> &geometric_entities_ids() const;

 private:
  const size_t id_;
  const std::string name_;
  std::vector<size_t> geometric_entities_ids_;
};

class GeometryData {
 public:
  void AddNode(size_t id, const std::array<double, 3> &coords);
  void AddGeometricEntityDataGroup(size_t id, const std::string &group_name);
  void AddGeometricEntityData(size_t id, size_t geometric_entity_type,
                              const std::vector<size_t> &node_ids,
                              size_t owner_shape_dimension,
                              size_t owner_shape_id);
  void RegisterShapeId(size_t dimensions, size_t shape_id,
                        size_t entity_group_id);
  const std::vector<NodeData> &nodes() const;
  const std::vector<GeometricEntityData> &geometric_entities() const;
  const std::vector<GeometricEntityDataGroup> &geometric_entities_groups()
      const;

 private:
  GeometricEntityDataGroup &GetGeometricEntityDataGroup(size_t group_id);

  std::vector<NodeData> nodes_;
  std::vector<GeometricEntityData> geometric_entities_;
  std::vector<GeometricEntityDataGroup> geometric_entities_groups_;
  std::array<std::unordered_map<size_t, std::vector<size_t>>, 4>
      shape_id_to_entities_group_ids_maps_;
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

class ShapesParser : public Parser {
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
