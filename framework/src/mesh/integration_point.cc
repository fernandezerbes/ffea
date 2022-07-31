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

IntegrationPointsGroupPtr QuadratureRule1x2::GetIntegrationPoints() const {
  IntegrationPointsGroupPtr integration_points =
      std::make_shared<std::vector<IntegrationPoint>>();
  integration_points->reserve(2);
  integration_points->push_back(
      IntegrationPoint(Coordinates({-0.5773502691896257, 0.0, 0.0}), 1.0));
  integration_points->push_back(
      IntegrationPoint(Coordinates({0.5773502691896257, 0.0, 0.0}), 1.0));
  return integration_points;
}

IntegrationPointsGroupPtr QuadratureRule2x2::GetIntegrationPoints() const {
  IntegrationPointsGroupPtr integration_points =
      std::make_shared<std::vector<IntegrationPoint>>();
  integration_points->reserve(4);
  integration_points->push_back(IntegrationPoint(
      Coordinates({-0.5773502691896257, -0.5773502691896257, 0.0}), 1.0));
  integration_points->push_back(IntegrationPoint(
      Coordinates({0.5773502691896257, -0.5773502691896257, 0.0}), 1.0));
  integration_points->push_back(IntegrationPoint(
      Coordinates({-0.5773502691896257, 0.5773502691896257, 0.0}), 1.0));
  integration_points->push_back(IntegrationPoint(
      Coordinates({0.5773502691896257, 0.5773502691896257, 0.0}), 1.0));
  return integration_points;
}

}  // namespace ffea