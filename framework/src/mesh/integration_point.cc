#include "../../inc/mesh/integration_point.h"

namespace ffea {

IntegrationPoint::IntegrationPoint(double x, double y, double z, double weight)
    : local_coords_(x, y, z), weight_(weight) {}

const Coordinates &IntegrationPoint::local_coords() const { return local_coords_; }

double IntegrationPoint::weight() const { return weight_; }

}  // namespace ffea
