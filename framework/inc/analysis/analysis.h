#ifndef FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_
#define FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_

#include "../fileio/output_writer.h"
#include "../model/model.h"

namespace ffea {

class Analysis {
 public:
  explicit Analysis(Model &model, OutputWriter writer);

  void Solve(const std::string &filename, const std::string &group_name);

 private:
  Model &model_;
  OutputWriter writer_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_
