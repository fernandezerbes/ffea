#include "../../inc/assembly/assembler.h"

namespace ffea {

Assembler::Assembler(const Mesh &mesh,
                     const ElementProcessor &element_processor)
    : mesh_(mesh), element_processor_(element_processor) {}

const std::pair<Eigen::MatrixXd, Eigen::VectorXd>
Assembler::ProcessLinearSystem() const {}

void Assembler::ScatterSolution(const Eigen::VectorXd &solution) {}

}  // namespace ffea