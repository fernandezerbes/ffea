
#include "../../inc/mesh/mesh_builder.h"

#include <stdexcept>

namespace ffea {

MeshBuilder::MeshBuilder(Geometry &geometry)
    : geometry_(geometry), element_factories_() {}

Mesh MeshBuilder::Build(size_t number_of_fields) const {
  Mesh mesh(geometry_, number_of_fields);

  for (auto &item : geometry_.geometric_entities_groups_) {
    const auto &group_name = item.first;
    const auto &element_factory = element_factories_.at(group_name);
    for (auto &geometric_entity : item.second) {
      mesh.AddElement(group_name, *geometric_entity, element_factory);
    }
  }

  std::cout << "Built mesh with " << mesh.number_of_dofs() << " dofs"
            << std::endl;

  return mesh;
}

void MeshBuilder::RegisterElementFactory(const std::string &group_name,
                                         const ElementFactory &factory) {
  if (element_factories_.contains(group_name)) {
    throw std::runtime_error("Factory already registered for group " +
                             group_name);
  }

  element_factories_.insert({group_name, factory});
}

}  // namespace ffea
