#include "../../inc/math/Utils.h"
#include "../../inc/mesh/Line2.h"

namespace ffea {

Line2::Line2(const std::vector<Node*> &nodes) : Element(nodes) {}

Line2::~Line2() {}

double Line2::EvaluateJacobianDet(
    const Coordinates &local_coordinates) const {
  return ComputeLength();
}

double Line2::ComputeLength() const {
  const auto &first_node_coordinates = nodes_[0]->coordinates();
  const auto &second_node_coordinates = nodes_[1]->coordinates();

  double length = utilities::DistanceBetweenPoints(
    first_node_coordinates.get(), second_node_coordinates.get());
  return length;
}

std::vector<double> Line2::EvaluateShapeFunctions(
    const Coordinates &local_coordinates) const {
  double xi = local_coordinates.get(0);
  double N0 = xi;
  double N1 = 1.0 - xi;
  return {N0, N1};
}

} // namespace ffea