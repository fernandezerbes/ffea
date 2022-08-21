#ifndef FFEA_FRAMEWORK_INC_MESH_MESHBUILDER_H_
#define FFEA_FRAMEWORK_INC_MESH_MESHBUILDER_H_

#include <string>
#include <fstream>

#include "./mesh.h"
#include "../fileio/mesh_parser.h"

namespace ffea {

class MeshBuilder {
 public:
  Mesh Build(size_t number_of_dofs_per_node);

 protected:
  virtual void AddNodes(Mesh &mesh) = 0;
  virtual void AddElements(Mesh &mesh) = 0;
};

class MeshFromFileBuilder : public MeshBuilder {
 public:
  MeshFromFileBuilder(const std::string &file_path);
  ~MeshFromFileBuilder();

 protected:
  virtual void AddNodes(Mesh &mesh) override;
  virtual void AddElements(Mesh &mesh) override;

 private:
  std::ifstream file_stream_;
  MeshParser parser_;
  MeshData mesh_data_;
};

class CartesianMeshBuilder : public MeshBuilder {
 public:
  CartesianMeshBuilder(double x_length, double y_length, size_t elements_in_x,
                       size_t elements_in_y);
  ~CartesianMeshBuilder();

 protected:
  virtual void AddNodes(Mesh &mesh) override;
  virtual void AddElements(Mesh &mesh) override;

 private:
  double x_length_;
  double y_length_;
  size_t elements_in_x_;
  size_t elements_in_y_;
  size_t nodes_in_x_;
  size_t nodes_in_y_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_MESHBUILDER_H_
