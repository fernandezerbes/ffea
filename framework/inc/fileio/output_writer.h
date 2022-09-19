#ifndef FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_
#define FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_

#include <string>

#include "../mesh/mesh.h"
#include "../postprocessor/postprocessor.h"

namespace ffea {

class OutputWriter {
 public:
  explicit OutputWriter(Mesh& mesh);
  void RegisterPostProcessor(const PostProcessor &postprocessor);
  void WriteQuad(const std::string& filename) const;
  void WriteTria(const std::string& filename) const;
  void WriteTetra(const std::string& filename) const;
  void WriteTetraQuadratic(const std::string& filename) const;

 private:
  std::vector<const PostProcessor*> postprocessors_;
  const Mesh& mesh_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_