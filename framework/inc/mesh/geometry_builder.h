#ifndef FFEA_FRAMEWORK_INC_MESH_GEOMETRYBUILDER_H_
#define FFEA_FRAMEWORK_INC_MESH_GEOMETRYBUILDER_H_

#include <fstream>
#include <string>

#include "../fileio/geometry_parser.h"
#include "./geometric_entity_factory.h"
#include "./geometry.h"

namespace ffea {

class GeometryBuilder {
 public:
  GeometryBuilder(const GeometricEntityFactory &factory);
  Geometry Build();

 protected:
  virtual void AddNodes(Geometry &geometry) = 0;
  virtual void AddEntities(Geometry &geometry) = 0;
  const GeometricEntityFactory &geometric_entity_factory_;
};

class GeometryFromFileBuilder : public GeometryBuilder {
 public:
  GeometryFromFileBuilder(
      const std::string &file_path,
      const GeometricEntityFactory &factory);
  ~GeometryFromFileBuilder();

 protected:
  virtual void AddNodes(Geometry &geometry) override;
  virtual void AddEntities(Geometry &geometry) override;

 private:
  std::ifstream file_stream_;
  GeometryParser parser_;
  GeometryData mesh_data_;
};

// class CartesianGeometryBuilder : public GeometryBuilder {
//  public:
//   CartesianGeometryBuilder(double x_length, double y_length,
//                            size_t elements_in_x, size_t elements_in_y);
//   ~CartesianGeometryBuilder();

//  protected:
//   virtual void AddNodes(Geometry &geometry) override;
//   virtual void AddEntities(Geometry &geometry) override;

//  private:
//   double x_length_;
//   double y_length_;
//   size_t elements_in_x_;
//   size_t elements_in_y_;
//   size_t nodes_in_x_;
//   size_t nodes_in_y_;
// };

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_GEOMETRYBUILDER_H_
