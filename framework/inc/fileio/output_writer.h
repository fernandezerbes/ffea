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
  explicit OutputWriter(Mesh& mesh);

  void RegisterPostProcessor(const PostProcessor& postprocessor);
  void Write(const std::string& filename, const std::string& group_name) const;

 private:
  std::vector<const PostProcessor*> postprocessors_;
  const Mesh& mesh_;
};

vtu11::VtkIndexType MapToVtkNodeIdx(GeometricEntityType entity_type,
                                    size_t node_idx);

const std::unordered_map<GeometricEntityType, vtu11::VtkCellType>
    geometric_entity_to_vtk_cell_map = {
        {GeometricEntityType::kTwoNodeLine, 3},
        {GeometricEntityType::kThreeNodeTria, 5},
        {GeometricEntityType::kFourNodeQuad, 8},
        {GeometricEntityType::kFourNodeTetra, 10},
        {GeometricEntityType::kEightNodeHex, 12},
        {GeometricEntityType::kSixNodePrism, 13},
        {GeometricEntityType::kFiveNodePiramid, 14},
        {GeometricEntityType::kThreeNodeLine, 21},
        {GeometricEntityType::kSixNodeTria, 22},
        {GeometricEntityType::kNineNodeQuad, 23},
        {GeometricEntityType::kTenNodeTetra, 24},
        {GeometricEntityType::kTwentySevenNodeHex, 25}};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_
