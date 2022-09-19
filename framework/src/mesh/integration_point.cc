#include "../../inc/mesh/integration_point.h"

#include <memory>

namespace ffea {

IntegrationPoint::IntegrationPoint(double x, double y, double z, double weight)
 : local_coordinates_(x, y, z), weight_(weight) {}

const Coordinates &IntegrationPoint::local_coordinates() const {
  return local_coordinates_;
}

double IntegrationPoint::weight() const { return weight_; }

}  // namespace ffea
