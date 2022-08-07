#ifndef FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_
#define FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_

#include <utility>

#include "../mesh/mesh.h"

namespace ffea {

class Assembler {
public:
  Assembler(const Mesh &mesh);

  const std::pair<Eigen::MatrixXd, Eigen::VectorXd> ProcessLinearSystem() const;
  void ScatterSolution(const Eigen::VectorXd &solution);

private:
 const Mesh &mesh_;
};

}

#endif  // FFEA_FRAMEWORK_INC_ASSEMBLY_ASSEMBLER_H_
