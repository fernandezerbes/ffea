#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_

#include <array>
#include <memory>

#include "./coordinates.h"
#include "./geometric_entity.h"

namespace ffea {

class IntegrationPoint {
 public:
  IntegrationPoint();
  IntegrationPoint(const Coordinates &local_coordinates, double weight);
  IntegrationPoint(double x, double y, double z, double weight);
  ~IntegrationPoint();

  const Coordinates &local_coordinates() const;
  double weight() const;

 private:
  Coordinates local_coordinates_;
  double weight_;
};

using IntegrationPointsGroup = std::vector<IntegrationPoint>;

// Lines

const IntegrationPointsGroup rule_line_1 = {{0.0, 0.0, 0.0, 2.0}};

const IntegrationPointsGroup rule_line_2 = {
    {-0.5773502691896257, 0.0, 0.0, 1.0}, {0.5773502691896257, 0.0, 0.0, 1.0}};

const IntegrationPointsGroup rule_line_3 = {
    {0.0, 0.0, 0.0, 0.8888888888888888},
    {-0.7745966692414834, 0.0, 0.0, 0.5555555555555556},
    {0.0, 0.7745966692414834, 0.0, 0.5555555555555556}};

// Trias

const IntegrationPointsGroup rule_tria_1 = {{1.0 / 3.0, 1.0 / 3.0, 0.0, 0.5}};

const IntegrationPointsGroup rule_tria_3 = {{0.5, 0.5, 0.0, 1.0 / 6.0},
                                            {0.5, 0.0, 0.0, 1.0 / 6.0},
                                            {0.0, 0.5, 0.0, 1.0 / 6.0}};

const IntegrationPointsGroup rule_tria_4 = {
    {1.0 / 3.0, 1.0 / 3.0, 0.0, 9.0 / 32.0},
    {0.6, 0.2, 0.0, 25.0 / 96.0},
    {0.2, 0.6, 0.0, 25.0 / 96.0},
    {0.2, 0.2, 0.0, 25.0 / 96.0}};

// Quads

const IntegrationPointsGroup rule_quad_1 = {{0.0, 0.0, 0.0, 1.0}};

const IntegrationPointsGroup rule_quad_4 = {
    {-0.5773502691896257, -0.5773502691896257, 0.0, 1.0},
    {0.5773502691896257, -0.5773502691896257, 0.0, 1.0},
    {-0.5773502691896257, 0.5773502691896257, 0.0, 1.0},
    {0.5773502691896257, 0.5773502691896257, 0.0, 1.0}};

// Tetras

const IntegrationPointsGroup rule_tetra_1 = {
    {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0, 0.5}};

// Hexas

const IntegrationPointsGroup rule_hex_8 = {
    {-0.5773502691896257, -0.5773502691896257, -0.5773502691896257, 1.0},
    {0.5773502691896257, -0.5773502691896257, -0.5773502691896257, 1.0},
    {-0.5773502691896257, 0.5773502691896257, -0.5773502691896257, 1.0},
    {0.5773502691896257, 0.5773502691896257, -0.5773502691896257, 1.0},
    {-0.5773502691896257, -0.5773502691896257, 0.5773502691896257, 1.0},
    {0.5773502691896257, -0.5773502691896257, 0.5773502691896257, 1.0},
    {-0.5773502691896257, 0.5773502691896257, 0.5773502691896257, 1.0},
    {0.5773502691896257, 0.5773502691896257, 0.5773502691896257, 1.0}};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
