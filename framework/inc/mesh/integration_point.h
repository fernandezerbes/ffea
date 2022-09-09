#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_

#include <array>
#include <memory>

#include "./coordinates.h"

namespace ffea {

class IntegrationPoint {
 public:
  IntegrationPoint();
  IntegrationPoint(const Coordinates &local_coordinates, double weight);
  ~IntegrationPoint();

  const Coordinates &local_coordinates() const;
  double weight() const;

 private:
  Coordinates local_coordinates_;
  double weight_;
};

using Quadrature = std::vector<IntegrationPoint>;

const Quadrature rule_line_2 = {
    IntegrationPoint(Coordinates({-0.5773502691896257, 0.0, 0.0}), 1.0),
    IntegrationPoint(Coordinates({0.5773502691896257, 0.0, 0.0}), 1.0)};

const Quadrature rule_quad_4 = {
    IntegrationPoint(
        Coordinates({-0.5773502691896257, -0.5773502691896257, 0.0}), 1.0),
    IntegrationPoint(
        Coordinates({0.5773502691896257, -0.5773502691896257, 0.0}), 1.0),
    IntegrationPoint(
        Coordinates({-0.5773502691896257, 0.5773502691896257, 0.0}), 1.0),
    IntegrationPoint(Coordinates({0.5773502691896257, 0.5773502691896257, 0.0}),
                     1.0)};

const Quadrature rule_tria_1 = {
    IntegrationPoint(Coordinates({1.0 / 3.0, 1.0 / 3.0, 0.0}), 0.5)};

const Quadrature rule_tria_3 = {
    IntegrationPoint(Coordinates({0.5, 0.5, 0.0}), 1.0 / 6.0),
    IntegrationPoint(Coordinates({0.5, 0.0, 0.0}), 1.0 / 6.0),
    IntegrationPoint(Coordinates({0.0, 0.5, 0.0}), 1.0 / 6.0)};

const Quadrature rule_tetra_1 = {
    IntegrationPoint(Coordinates({1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}), 0.5)};

const Quadrature rule_hex_8 = {
    IntegrationPoint(Coordinates({-0.5773502691896257, -0.5773502691896257,
                                  -0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({0.5773502691896257, -0.5773502691896257,
                                  -0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({-0.5773502691896257, 0.5773502691896257,
                                  -0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({0.5773502691896257, 0.5773502691896257,
                                  -0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({-0.5773502691896257, -0.5773502691896257,
                                  0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({0.5773502691896257, -0.5773502691896257,
                                  0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({-0.5773502691896257, 0.5773502691896257,
                                  0.5773502691896257}),
                     1.0),
    IntegrationPoint(Coordinates({0.5773502691896257, 0.5773502691896257,
                                  0.5773502691896257}),
                     1.0)};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
