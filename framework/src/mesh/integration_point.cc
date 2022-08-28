#include "../../inc/mesh/integration_point.h"

#include <memory>

namespace ffea {

IntegrationPoint::IntegrationPoint() {}

IntegrationPoint::IntegrationPoint(const Coordinates &local_coordinates,
                                   double weight)
    : local_coordinates_(local_coordinates), weight_(weight) {}

IntegrationPoint::~IntegrationPoint() {}

const Coordinates &IntegrationPoint::local_coordinates() const {
  return local_coordinates_;
}

double IntegrationPoint::weight() const { return weight_; }

std::vector<IntegrationPoint> QuadratureRule1x2::GetIntegrationPoints() const {
  std::vector<IntegrationPoint> integration_points;
  integration_points.reserve(2);
  integration_points.push_back(
      IntegrationPoint(Coordinates({-0.5773502691896257, 0.0, 0.0}), 1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.5773502691896257, 0.0, 0.0}), 1.0));

  return integration_points;
}

std::vector<IntegrationPoint> QuadratureRule2x2::GetIntegrationPoints() const {
  std::vector<IntegrationPoint> integration_points;
  integration_points.reserve(4);
  integration_points.push_back(IntegrationPoint(
      Coordinates({-0.5773502691896257, -0.5773502691896257, 0.0}), 1.0));
  integration_points.push_back(IntegrationPoint(
      Coordinates({0.5773502691896257, -0.5773502691896257, 0.0}), 1.0));
  integration_points.push_back(IntegrationPoint(
      Coordinates({-0.5773502691896257, 0.5773502691896257, 0.0}), 1.0));
  integration_points.push_back(IntegrationPoint(
      Coordinates({0.5773502691896257, 0.5773502691896257, 0.0}), 1.0));
  return integration_points;
}

}  // namespace ffea