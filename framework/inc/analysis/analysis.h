#ifndef FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_
#define FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_

#include <eigen3/Eigen/Dense>

#include "../model/model.h"

namespace ffea {

class Analysis {
 public:
  explicit Analysis(Model &model);

  void Solve();

 private:
  Model &model_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_ANALYSIS_ANALYSIS_H_
