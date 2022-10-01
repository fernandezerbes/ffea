#include "../../inc/fileio/output_writer.h"

#include <fstream>

namespace ffea {

OutputWriter::OutputWriter(Mesh& mesh) : mesh_(mesh) {}

void OutputWriter::RegisterPostProcessor(const PostProcessor& postprocessor) {
  postprocessors_.push_back(&postprocessor);
}

void OutputWriter::WriteQuad(const std::string& filename) const {
  std::ofstream file;

  file.open(filename);

  file << "# vtk DataFile Version 4.2" << std::endl;
  file << "Test Data                 " << std::endl;
  file << "ASCII                     " << std::endl;
  file << "DATASET UNSTRUCTURED_GRID " << std::endl;

  file << "POINTS " << mesh_.number_of_nodes() << " "
       << "double " << std::endl;

  for (const auto& node : mesh_.nodes()) {
    const auto& coords = node.coords();
    file << coords.get(0) << "\t" << coords.get(1) << "\t"
         << coords.get(2) << std::endl;
  }

  auto& body_elements = mesh_.GetElementGroup("surface");

  size_t integers_per_element = 5;
  size_t cell_list_size = body_elements.size() * integers_per_element;

  file << "CELLS " << body_elements.size() << " " << cell_list_size
       << std::endl;

  for (auto& element : body_elements) {
    file << element.GetNumberOfNodes() << "\t";
    for (auto& node : element.nodes()) {
      file << node->id() << "\t";
    }
    file << std::endl;
  }

  file << "CELL_TYPES " << body_elements.size() << std::endl;
  size_t cell_type = 9;

  for (size_t i = 0; i < body_elements.size(); i++) {
    file << cell_type << std::endl;
  }

  file << "POINT_DATA " << mesh_.number_of_nodes() << std::endl;
  file << "VECTORS displacements double" << std::endl;
  // file << "LOOKUP_TABLE default" << std::endl;

  for (size_t dof_id = 0; dof_id < mesh_.number_of_dofs();
       dof_id += mesh_.dofs_per_node()) {
    file << mesh_.GetSolutionAtDof(dof_id) << "\t"
         << mesh_.GetSolutionAtDof(dof_id + 1) << "\t0" << std::endl;
  }

  file.close();
}

void OutputWriter::WriteTria(const std::string& filename) const {
  std::ofstream file;

  file.open(filename);

  file << "# vtk DataFile Version 4.2" << std::endl;
  file << "Test Data                 " << std::endl;
  file << "ASCII                     " << std::endl;
  file << "DATASET UNSTRUCTURED_GRID " << std::endl;

  file << "POINTS " << mesh_.number_of_nodes() << " "
       << "double " << std::endl;

  for (const auto& node : mesh_.nodes()) {
    const auto& coords = node.coords();
    file << coords.get(0) << "\t" << coords.get(1) << "\t"
         << coords.get(2) << std::endl;
  }

  auto& body_elements = mesh_.GetElementGroup("surface");

  size_t integers_per_element = 4;
  size_t cell_list_size = body_elements.size() * integers_per_element;

  file << "CELLS " << body_elements.size() << " " << cell_list_size
       << std::endl;

  for (auto& element : body_elements) {
    file << element.GetNumberOfNodes() << "\t";
    for (auto& node : element.nodes()) {
      file << node->id() << "\t";
    }
    file << std::endl;
  }

  file << "CELL_TYPES " << body_elements.size() << std::endl;
  size_t cell_type = 5;

  for (size_t i = 0; i < body_elements.size(); i++) {
    file << cell_type << std::endl;
  }

  file << "POINT_DATA " << mesh_.number_of_nodes() << std::endl;
  file << "VECTORS displacements double" << std::endl;
  // file << "LOOKUP_TABLE default" << std::endl;

  for (size_t dof_id = 0; dof_id < mesh_.number_of_dofs();
       dof_id += mesh_.dofs_per_node()) {
    file << mesh_.GetSolutionAtDof(dof_id) << "\t"
         << mesh_.GetSolutionAtDof(dof_id + 1) << "\t0" << std::endl;
  }

  file.close();
}

void OutputWriter::WriteTetra(const std::string& filename) const {
  std::ofstream file;

  file.open(filename);

  file << "# vtk DataFile Version 4.2" << std::endl;
  file << "Test Data                 " << std::endl;
  file << "ASCII                     " << std::endl;
  file << "DATASET UNSTRUCTURED_GRID " << std::endl;

  file << "POINTS " << mesh_.number_of_nodes() << " "
       << "double " << std::endl;

  for (const auto& node : mesh_.nodes()) {
    const auto& coords = node.coords();
    file << coords.get(0) << "\t" << coords.get(1) << "\t"
         << coords.get(2) << std::endl;
  }

  auto& body_elements = mesh_.GetElementGroup("body");

  size_t integers_per_element = 5;
  size_t cell_list_size = body_elements.size() * integers_per_element;

  file << "CELLS " << body_elements.size() << " " << cell_list_size
       << std::endl;

  for (auto& element : body_elements) {
    file << element.GetNumberOfNodes() << "\t";
    for (auto& node : element.nodes()) {
      file << node->id() << "\t";
    }
    file << std::endl;
  }

  file << "CELL_TYPES " << body_elements.size() << std::endl;
  size_t cell_type = 10;

  for (size_t i = 0; i < body_elements.size(); i++) {
    file << cell_type << std::endl;
  }

  file << "POINT_DATA " << mesh_.number_of_nodes() << std::endl;
  file << "VECTORS displacements double" << std::endl;
  // file << "LOOKUP_TABLE default" << std::endl;

  for (size_t dof_id = 0; dof_id < mesh_.number_of_dofs();
       dof_id += mesh_.dofs_per_node()) {
    file << mesh_.GetSolutionAtDof(dof_id) << "\t"
         << mesh_.GetSolutionAtDof(dof_id + 1) << "\t"
         << mesh_.GetSolutionAtDof(dof_id + 2) << std::endl;
  }

  file.close();
}

void OutputWriter::WriteTetraQuadratic(const std::string& filename) const {
  std::ofstream file;

  file.open(filename);

  file << "# vtk DataFile Version 4.2" << std::endl;
  file << "Test Data                 " << std::endl;
  file << "ASCII                     " << std::endl;
  file << "DATASET UNSTRUCTURED_GRID " << std::endl;

  file << "POINTS " << mesh_.number_of_nodes() << " "
       << "double " << std::endl;

  for (const auto& node : mesh_.nodes()) {
    const auto& coords = node.coords();
    file << coords.get(0) << "\t" << coords.get(1) << "\t"
         << coords.get(2) << std::endl;
  }

  auto& body_elements = mesh_.GetElementGroup("body");

  size_t integers_per_element = 11;
  size_t cell_list_size = body_elements.size() * integers_per_element;

  file << "CELLS " << body_elements.size() << " " << cell_list_size
       << std::endl;

  for (auto& element : body_elements) {
    file << element.GetNumberOfNodes() << "\t";
    auto vtk_nodes = element.nodes();
    std::swap(vtk_nodes[8], vtk_nodes[9]);    // Nodes 8 and 9 of vtk and Gmesh are different
    for (auto& node : vtk_nodes) {
      file << node->id() << "\t";
    }
    file << std::endl;
  }

  file << "CELL_TYPES " << body_elements.size() << std::endl;
  size_t cell_type = 24;

  for (size_t i = 0; i < body_elements.size(); i++) {
    file << cell_type << std::endl;
  }

  file << "POINT_DATA " << mesh_.number_of_nodes() << std::endl;
  file << "VECTORS displacements double" << std::endl;
  // file << "LOOKUP_TABLE default" << std::endl;

  for (size_t dof_id = 0; dof_id < mesh_.number_of_dofs();
       dof_id += mesh_.dofs_per_node()) {
    file << mesh_.GetSolutionAtDof(dof_id) << "\t"
         << mesh_.GetSolutionAtDof(dof_id + 1) << "\t"
         << mesh_.GetSolutionAtDof(dof_id + 2) << std::endl;
  }

  file.close();
}

}  // namespace ffea
