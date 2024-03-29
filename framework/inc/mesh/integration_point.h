#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_

#include "../geometry/coordinates.h"
#include "../geometry/geometric_entity.h"

namespace ffea {

class IntegrationPoint {
 public:
  IntegrationPoint(double x, double y, double z, double weight);

  const Coordinates &local_coords() const;
  double weight() const;

 private:
  Coordinates local_coords_;
  double weight_;
};

using IntegrationPointsGroup = std::vector<IntegrationPoint>;
using IntegrationPointsTable = std::vector<IntegrationPointsGroup>;
const IntegrationPointsGroup dummy_rule = {};

// Points

const IntegrationPointsGroup rule_point_1 = {{0.0, 0.0, 0.0, 1.0}};

// Lines

const IntegrationPointsGroup rule_line_1 = {{0.0, 0.0, 0.0, 2.0}};

const IntegrationPointsGroup rule_line_2 = {{-0.5773502691896257, 0.0, 0.0, 1.0},
                                            {0.5773502691896257, 0.0, 0.0, 1.0}};

const IntegrationPointsGroup rule_line_3 = {{0.0, 0.0, 0.0, 0.8888888888888888},
                                            {-0.7745966692414834, 0.0, 0.0, 0.5555555555555556},
                                            {0.0, 0.7745966692414834, 0.0, 0.5555555555555556}};

// Trias

const IntegrationPointsGroup rule_tria_1 = {{1.0 / 3.0, 1.0 / 3.0, 0.0, 0.5}};

const IntegrationPointsGroup rule_tria_3 = {
    {0.5, 0.5, 0.0, 1.0 / 6.0}, {0.5, 0.0, 0.0, 1.0 / 6.0}, {0.0, 0.5, 0.0, 1.0 / 6.0}};

const IntegrationPointsGroup rule_tria_4 = {{1.0 / 3.0, 1.0 / 3.0, 0.0, 9.0 / 32.0},
                                            {0.6, 0.2, 0.0, 25.0 / 96.0},
                                            {0.2, 0.6, 0.0, 25.0 / 96.0},
                                            {0.2, 0.2, 0.0, 25.0 / 96.0}};

// Quads

const IntegrationPointsGroup rule_quad_1 = {{0.0, 0.0, 0.0, 1.0}};

const IntegrationPointsGroup rule_quad_4 = {{-0.5773502691896257, -0.5773502691896257, 0.0, 1.0},
                                            {0.5773502691896257, -0.5773502691896257, 0.0, 1.0},
                                            {-0.5773502691896257, 0.5773502691896257, 0.0, 1.0},
                                            {0.5773502691896257, 0.5773502691896257, 0.0, 1.0}};

// Tetras

const IntegrationPointsGroup rule_tetra_1 = {{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0, 0.5}};

const IntegrationPointsGroup rule_tetra_4 = {{0.58541020, 0.13819660, 0.13819660, 0.041666667},
                                             {0.13819660, 0.58541020, 0.13819660, 0.041666667},
                                             {0.13819660, 0.13819660, 0.58541020, 0.041666667},
                                             {0.13819660, 0.13819660, 0.13819660, 0.041666667}};

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

// Lookup tables {GeometricEntityType --> IntegrationPointsGroup}

const IntegrationPointsTable full_integration_points = {
    /* kTwoNodeLine */ rule_line_1,
    /* kThreeNodeTria */ rule_tria_1,
    /* kFourNodeQuad */ rule_quad_4,
    /* kFourNodeTetra */ rule_tetra_1,
    /* kEightNodeHex */ rule_hex_8,
    /* kSixNodePrism */ dummy_rule,
    /* kFiveNodePiramid */ dummy_rule,
    /* kThreeNodeLine */ dummy_rule,
    /* kSixNodeTria */ rule_tria_3,
    /* kNineNodeQuad */ dummy_rule,
    /* kTenNodeTetra */ rule_tetra_4,
    /* kTwentySevenNodeHex */ dummy_rule,
    /* kEighteenNodePrism */ dummy_rule,
    /* kFourteenNodePiramid */ dummy_rule,
    /* kOneNodePoint */ rule_point_1,
    /* kEightNodeQuad */ dummy_rule,
    /* kTwentyNodeHex */ dummy_rule,
    /* kFifteenNodePrism */ dummy_rule,
    /* kThirteenNodePiramid */ dummy_rule,
};

const IntegrationPointsTable reduced_integration_points = {
    /* kTwoNodeLine */ rule_line_1,
    /* kThreeNodeTria */ rule_tria_1,
    /* kFourNodeQuad */ rule_quad_1,
    /* kFourNodeTetra */ rule_tetra_1,
    /* kEightNodeHex */ rule_hex_8,
    /* kSixNodePrism */ dummy_rule,
    /* kFiveNodePiramid */ dummy_rule,
    /* kThreeNodeLine */ dummy_rule,
    /* kSixNodeTria */ rule_tria_1,
    /* kNineNodeQuad */ dummy_rule,
    /* kTenNodeTetra */ rule_tetra_1,
    /* kTwentySevenNodeHex */ dummy_rule,
    /* kEighteenNodePrism */ dummy_rule,
    /* kFourteenNodePiramid */ dummy_rule,
    /* kOneNodePoint */ rule_point_1,
    /* kEightNodeQuad */ dummy_rule,
    /* kTwentyNodeHex */ dummy_rule,
    /* kFifteenNodePrism */ dummy_rule,
    /* kThirteenNodePiramid */ dummy_rule,
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
