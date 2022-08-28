#ifndef FFEA_FRAMEWORK_INC_MESH_MESHBUILDER_H_
#define FFEA_FRAMEWORK_INC_MESH_MESHBUILDER_H_

#include <string>
#include <unordered_map>

#include "./element_factory.h"
#include "./geometry.h"
#include "./mesh.h"

namespace ffea {

class MeshBuilder

{
 public:
  MeshBuilder(Geometry &geometry);
  ~MeshBuilder();

  Mesh Build(size_t number_of_fields) const;
  void RegisterElementFactory(const std::string &group_name,
                              const ElementFactory &factory);

 private:
  std::unordered_map<std::string, ElementFactory> element_factories_;
  Geometry &geometry_;
};

}  // namespace ffea

#endif // FFEA_FRAMEWORK_INC_MESH_MESHBUILDER_H_
