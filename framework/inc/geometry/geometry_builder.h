#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRYBUILDER_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRYBUILDER_H_

#include <fstream>
#include <string>

#include "../fileio/geometry_parser.h"
#include "./geometric_entity_factory.h"
#include "./geometry.h"

namespace ffea {

class GeometryBuilder {
 public:
  explicit GeometryBuilder(const GeometricEntityFactory &factory);
  
  Geometry Build();

 protected:
  const GeometricEntityFactory &factory_;
 
 private:
  virtual void AddNodes(Geometry &geometry) = 0;
  virtual void AddEntities(Geometry &geometry) = 0;
  virtual void AddEntitiesGroups(Geometry &geometry) = 0;
};

class GeometryFromFileBuilder : public GeometryBuilder {
 public:
  GeometryFromFileBuilder(const std::string &file_path,
                          const GeometricEntityFactory &factory);

 private:
  virtual void AddNodes(Geometry &geometry) override;
  virtual void AddEntities(Geometry &geometry) override;
  virtual void AddEntitiesGroups(Geometry &geometry) override;

  std::ifstream file_stream_;
  GeometryParser parser_;
  GeometryData data_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRYBUILDER_H_
