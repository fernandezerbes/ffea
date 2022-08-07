#include "../../inc/assembly/assembler.h"

namespace ffea {

Assembler::Assembler(const Mesh &mesh) : mesh_(mesh) {}

const std::pair<Eigen::MatrixXd, Eigen::VectorXd>
Assembler::ProcessLinearSystem() const {}

void Assembler::ScatterSolution(const Eigen::VectorXd &solution) {}

}  // namespace ffea