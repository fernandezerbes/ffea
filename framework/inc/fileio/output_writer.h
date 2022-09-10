#ifndef FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_
#define FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_

#include <string>

#include "../mesh/mesh.h"
#include "../postprocessor/postprocessor.h"

namespace ffea {

class OutputWriter {
 public:
  OutputWriter(Mesh& mesh);
  void RegisterPostProcessor(const PostProcessor &postprocessor);
  void WriteQuad(const std::string& filename);
  void WriteTria(const std::string& filename);
  void WriteTetra(const std::string& filename);

 private:
  std::vector<const PostProcessor*> postprocessors_;
  const Mesh& mesh_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_FILEIO_WRITER_H_