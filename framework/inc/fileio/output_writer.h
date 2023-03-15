#ifndef FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_
#define FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_

#include <string>
#include <unordered_map>

#include "../geometry/geometric_entity.h"
#include "../mesh/mesh.h"
#include "../postprocessor/postprocessor.h"
#include "vtu11/vtu11.hpp"

namespace ffea {

class OutputWriter {
 public:
  explicit OutputWriter(Mesh &mesh);

  void RegisterPostProcessor(const PostProcessor &postprocessor);
  void Write(const std::string &filename, const std::string &group_name) const;

 private:
  std::vector<const PostProcessor *> postprocessors_;
  const Mesh &mesh_;
};

// clang-format off
// Map the entity type to the vtk cell number, since the ordering differs
const std::unordered_map<GeometricEntityType, vtu11::VtkCellType> entity_type_to_vtk_cell_map = {
    {GeometricEntityType::kTwoNodeLine, 3},
    {GeometricEntityType::kThreeNodeTria, 5},
    {GeometricEntityType::kFourNodeQuad, 9},
    {GeometricEntityType::kFourNodeTetra, 10},
    {GeometricEntityType::kEightNodeHex, 12},
    {GeometricEntityType::kSixNodePrism, 13},
    {GeometricEntityType::kFiveNodePiramid, 14},
    {GeometricEntityType::kThreeNodeLine, 21},
    {GeometricEntityType::kSixNodeTria, 22},
    {GeometricEntityType::kNineNodeQuad, 28},
    {GeometricEntityType::kTenNodeTetra, 24},
    {GeometricEntityType::kTwentySevenNodeHex, 25}
};

// Map the entity type to the map of ffea node indices to vtk node indices (node ordering differs)
const std::unordered_map<GeometricEntityType, std::vector<size_t>> ffea_nodes_to_vtk_nodes_maps = {
    {GeometricEntityType::kSixNodePrism, {0, 2, 1, 3, 5, 4}},
    {GeometricEntityType::kTenNodeTetra, {0, 1, 2, 3, 4, 5, 6, 7, 9, 8}},
};
// clang-format on

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_
