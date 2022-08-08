

#include "../../inc/fileio/output_writer.h"

namespace ffea {

OutputWriter::OutputWriter(Mesh& mesh) : mesh_(mesh) {}

void OutputWriter::RegisterPostProcessor(const PostProcessor& postprocessor) {
  postprocessors_.push_back(&postprocessor);
}

void OutputWriter::Write(const std::string& filename) {
  for (const auto& postprocessor: postprocessors_) {
    for (auto& element :
        mesh_.GetElementGroup(ElementGroupType::kBodyElements, "body")) {
      std::cout << postprocessor->GetValuesAtNode(element, 1) << std::endl;
    }
  }
}

}  // namespace ffea
