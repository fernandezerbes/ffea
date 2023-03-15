#include "../../inc/fileio/output_writer.h"

#include <fstream>

namespace ffea {

OutputWriter::OutputWriter(Mesh& mesh) : mesh_(mesh) {}

void OutputWriter::RegisterPostProcessor(const PostProcessor& postprocessor) {
  postprocessors_.push_back(&postprocessor);
}

void OutputWriter::Write(const std::string& filename, const std::string& group_name) const {
  std::vector<double> points;
  for (const auto& node : mesh_.nodes()) {  // TODO This should only work with
                                            // nodes of the element group
    points.push_back(node.coords().get(0));
    points.push_back(node.coords().get(1));
    points.push_back(node.coords().get(2));
  }

  std::vector<vtu11::VtkIndexType> connectivity;
  std::vector<vtu11::VtkIndexType> offsets;
  std::vector<vtu11::VtkCellType> types;
  vtu11::VtkIndexType offset = 0;
  for (const auto& element : mesh_.element_group(group_name)) {
    offset += element.number_of_nodes();
    offsets.push_back(offset);

    const auto& entity_type = element.geometric_entity_type();
    const auto& vtk_cell_type = entity_type_to_vtk_cell_map.at(entity_type);
    types.push_back(vtk_cell_type);

    const std::vector<size_t>* map = nullptr;
    if (ffea_nodes_to_vtk_nodes_maps.contains(entity_type)) {
      map = &(ffea_nodes_to_vtk_nodes_maps.at(entity_type));
    }

    for (size_t node_idx = 0; node_idx < element.number_of_nodes(); node_idx++) {
      auto vtk_node_idx = node_idx;
      if (map != nullptr) {
        vtk_node_idx = map->at(node_idx);
      }
      connectivity.push_back(element.node_tag(vtk_node_idx));
    }
  }

  vtu11::Vtu11UnstructuredMesh vtu_mesh{points, connectivity, offsets, types};

  std::vector<vtu11::DataSetData> data_set_data;
  data_set_data.reserve(postprocessors_.size());

  std::vector<vtu11::DataSetInfo> data_set_info;
  data_set_info.reserve(postprocessors_.size());

  for (const auto& postprocessor : postprocessors_) {
    data_set_data.push_back(postprocessor->Process(group_name));
    data_set_info.emplace_back(postprocessor->variable_name(), vtu11::DataSetType::PointData,
                               postprocessor->values_per_node());
  }

  vtu11::writeVtu(filename, vtu_mesh, data_set_info, data_set_data, "Ascii");
}

}  // namespace ffea
