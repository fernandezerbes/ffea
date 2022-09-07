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

std::vector<IntegrationPoint> QuadratureRule2x2x2::GetIntegrationPoints()
    const {
  std::vector<IntegrationPoint> integration_points;
  integration_points.reserve(8);
  integration_points.push_back(
      IntegrationPoint(Coordinates({-0.5773502691896257, -0.5773502691896257,
                                    -0.5773502691896257}),
                       1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.5773502691896257, -0.5773502691896257,
                                    -0.5773502691896257}),
                       1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({-0.5773502691896257, 0.5773502691896257,
                                    -0.5773502691896257}),
                       1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.5773502691896257, 0.5773502691896257,
                                    -0.5773502691896257}),
                       1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({-0.5773502691896257, -0.5773502691896257,
                                    0.5773502691896257}),
                       1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.5773502691896257, -0.5773502691896257,
                                    0.5773502691896257}),
                       1.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({-0.5773502691896257, 0.5773502691896257,
                                    0.5773502691896257}),
                       1.0));
  integration_points.push_back(IntegrationPoint(
      Coordinates({0.5773502691896257, 0.5773502691896257, 0.5773502691896257}),
      1.0));
  return integration_points;
}

std::vector<IntegrationPoint> QuadratureRuleTria1::GetIntegrationPoints()
    const {
  std::vector<IntegrationPoint> integration_points;
  integration_points.reserve(1);
  integration_points.push_back(
      IntegrationPoint(Coordinates({1.0 / 3.0, 1.0 / 3.0, 0.0}), 0.5));
  return integration_points;
}

std::vector<IntegrationPoint> QuadratureRuleTria3::GetIntegrationPoints()
    const {
  std::vector<IntegrationPoint> integration_points;
  integration_points.reserve(3);
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.5, 0.5, 0.0}), 1.0 / 6.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.5, 0.0, 0.0}), 1.0 / 6.0));
  integration_points.push_back(
      IntegrationPoint(Coordinates({0.0, 0.5, 0.0}), 1.0 / 6.0));
  return integration_points;
}

}  // namespace ffea