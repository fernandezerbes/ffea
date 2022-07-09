#include "../../inc/mesh/IntegrationPoint.h"

namespace ffea {

IntegrationPoint::IntegrationPoint(const Coordinates &local_coordinates,
                                   double weight)
  : local_coordinates_(local_coordinates),
    weight_(weight)
  {}

IntegrationPoint::~IntegrationPoint() {}

const Coordinates &IntegrationPoint::local_coordinates() const {
  return local_coordinates_;
}

double IntegrationPoint::weight() const {
  return weight_;
}

}