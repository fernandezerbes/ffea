
#include "../framework/inc/geometry/geometry.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../framework/inc/alias.h"
#include "../framework/inc/geometry/node.h"

using namespace testing;

class GeometryTest : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    geometry.AddNode({0.0, 0.0, 0.0});  // Node 0
    geometry.AddNode({1.0, 0.0, 0.0});  // Node 1
    geometry.AddNode({2.0, 0.0, 0.0});  // Node 2
    geometry.AddNode({0.0, 1.0, 0.0});  // Node 3
    geometry.AddNode({1.0, 1.0, 0.0});  // Node 4
    geometry.AddNode({2.0, 1.0, 0.0});  // Node 5
    geometry.AddNode({0.0, 2.0, 0.0});  // Node 6
    geometry.AddNode({1.0, 2.0, 0.0});  // Node 7
    geometry.AddNode({2.0, 2.0, 0.0});  // Node 8

    geometry.AddGeometricEntity(ffea::GeometricEntityType::kFourNodeQuad, {0, 1, 4, 3}, factory);
    geometry.AddGeometricEntity(ffea::GeometricEntityType::kFourNodeQuad, {1, 2, 5, 4}, factory);
    geometry.AddGeometricEntity(ffea::GeometricEntityType::kFourNodeQuad, {3, 4, 7, 6}, factory);
    geometry.AddGeometricEntity(ffea::GeometricEntityType::kThreeNodeTria, {4, 5, 7}, factory);
    geometry.AddGeometricEntity(ffea::GeometricEntityType::kThreeNodeTria, {5, 8, 7}, factory);
    geometry.AddGeometricEntity(ffea::GeometricEntityType::kTwoNodeLine, {6, 7}, factory);
    geometry.AddGeometricEntity(ffea::GeometricEntityType::kTwoNodeLine, {7, 8}, factory);

    geometry.RegisterGeometricEntityGroup("body", {0, 1, 2, 3, 4});
    geometry.RegisterGeometricEntityGroup("top", {5, 6});
  }

  ffea::Geometry geometry;
  ffea::GeometricEntityFactory2D factory;
};

TEST_F(GeometryTest, BasicExpectations) {
  ASSERT_THAT(geometry.number_of_nodes(), 9);
  ASSERT_THAT(geometry.nodes().size(), 9);
  ASSERT_THAT(geometry.entity_groups().size(), 2);
}

TEST_F(GeometryTest, GetEntityGroup) {
  auto entity_group = geometry.entity_group("top");

  // First entity
  ASSERT_THAT(entity_group[0]->type(), ffea::GeometricEntityType::kTwoNodeLine);

  auto first_node_coords = entity_group[0]->node_coords(0);
  auto second_node_coords = entity_group[0]->node_coords(1);

  EXPECT_DOUBLE_EQ(first_node_coords.get(0), 0.0);
  EXPECT_DOUBLE_EQ(first_node_coords.get(1), 2.0);
  EXPECT_DOUBLE_EQ(first_node_coords.get(2), 0.0);

  EXPECT_DOUBLE_EQ(second_node_coords.get(0), 1.0);
  EXPECT_DOUBLE_EQ(second_node_coords.get(1), 2.0);
  EXPECT_DOUBLE_EQ(second_node_coords.get(2), 0.0);

  // Second entity
  ASSERT_THAT(entity_group[1]->type(), ffea::GeometricEntityType::kTwoNodeLine);

  first_node_coords = entity_group[1]->node_coords(0);
  second_node_coords = entity_group[1]->node_coords(1);

  EXPECT_DOUBLE_EQ(first_node_coords.get(0), 1.0);
  EXPECT_DOUBLE_EQ(first_node_coords.get(1), 2.0);
  EXPECT_DOUBLE_EQ(first_node_coords.get(2), 0.0);

  EXPECT_DOUBLE_EQ(second_node_coords.get(0), 2.0);
  EXPECT_DOUBLE_EQ(second_node_coords.get(1), 2.0);
  EXPECT_DOUBLE_EQ(second_node_coords.get(2), 0.0);
}

TEST_F(GeometryTest, NodeAddition) {
  geometry.AddNode({0.0, 3.0, 0.0});
  ASSERT_THAT(geometry.number_of_nodes(), 10);
  EXPECT_THAT(geometry.nodes()[9].tag(), 9);
}

TEST_F(GeometryTest, GeometricEntityAddition) {
  geometry.AddGeometricEntity(ffea::GeometricEntityType::kThreeNodeTria, {0, 4, 3}, factory);
  geometry.RegisterGeometricEntityGroup("triangle", {7});
  auto *triangle = geometry.entity_group("triangle")[0];

  ASSERT_THAT(triangle->type(), ffea::GeometricEntityType::kThreeNodeTria);
}

TEST_F(GeometryTest, GeometricEntityGroupRegistration) {
  geometry.RegisterGeometricEntityGroup("top_left", {5});
  auto *entity = geometry.entity_group("top_left")[0];

  ASSERT_THAT(entity->type(), ffea::GeometricEntityType::kTwoNodeLine);

  auto first_node_coords = entity->node_coords(0);
  auto second_node_coords = entity->node_coords(1);

  EXPECT_DOUBLE_EQ(first_node_coords.get(0), 0.0);
  EXPECT_DOUBLE_EQ(first_node_coords.get(1), 2.0);
  EXPECT_DOUBLE_EQ(first_node_coords.get(2), 0.0);

  EXPECT_DOUBLE_EQ(second_node_coords.get(0), 1.0);
  EXPECT_DOUBLE_EQ(second_node_coords.get(1), 2.0);
  EXPECT_DOUBLE_EQ(second_node_coords.get(2), 0.0);
}
