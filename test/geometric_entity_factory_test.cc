#include "../framework/inc/geometry/geometric_entity_factory.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdexcept>

#include "../framework/inc/alias.h"
#include "../framework/inc/geometry/geometric_entity.h"
#include "../framework/inc/geometry/node.h"

using namespace testing;

class GeometricEntityFactory2DTest : public ::testing::Test {
 protected:
  GeometricEntityFactory2DTest()
      : dim(2),
        node0(24, {0.0, 0.0, 1.0}),
        node1(94, {1.0, 0.0, 1.0}),
        node2(1, {0.0, 0.0, 0.0}),
        node3(3, {0.0, 1.0, 1.0}),
        node4(7, {0.5, 0.0, 1.0}),
        node5(48, {0.5, 0.0, 0.5}),
        factory(ffea::GeometricEntityFactory2D()) {}

  size_t dim;
  ffea::Node node0;
  ffea::Node node1;
  ffea::Node node2;
  ffea::Node node3;
  ffea::Node node4;
  ffea::Node node5;
  const ffea::GeometricEntityFactory2D factory;
};

TEST_F(GeometricEntityFactory2DTest, GeometricEntityCreation) {
  // kTwoNodeLine
  auto entity = factory.CreateGeometricEntity(
      ffea::GeometricEntityType::kTwoNodeLine, {&node0, &node1});

  EXPECT_EQ(entity->type(), ffea::GeometricEntityType::kTwoNodeLine);
  EXPECT_EQ(entity->dim(), dim);

  // kThreeNodeTria
  entity = factory.CreateGeometricEntity(
      ffea::GeometricEntityType::kThreeNodeTria, {&node0, &node1, &node3});

  EXPECT_EQ(entity->type(), ffea::GeometricEntityType::kThreeNodeTria);
  EXPECT_EQ(entity->dim(), dim);

  // kFourNodeQuad
  entity =
      factory.CreateGeometricEntity(ffea::GeometricEntityType::kFourNodeQuad,
                                    {&node0, &node1, &node3, &node4});

  EXPECT_EQ(entity->type(), ffea::GeometricEntityType::kFourNodeQuad);
  EXPECT_EQ(entity->dim(), dim);

  // kSixNodeTria
  entity = factory.CreateGeometricEntity(
      ffea::GeometricEntityType::kSixNodeTria,
      {&node0, &node1, &node3, &node4, &node5, &node5});

  EXPECT_EQ(entity->type(), ffea::GeometricEntityType::kSixNodeTria);
  EXPECT_EQ(entity->dim(), dim);
}

TEST_F(GeometricEntityFactory2DTest, GeometricEntityNotSupported) {
  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kFourNodeTetra, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kEightNodeHex, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kSixNodePrism, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kFiveNodePiramid, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kThreeNodeLine, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kNineNodeQuad, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kTenNodeTetra, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kTwentySevenNodeHex, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kEighteenNodePrism, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kFourteenNodePiramid, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kOneNodePoint, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kEightNodeQuad, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kTwentyNodeHex, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kFifteenNodePrism, {}),
               std::runtime_error);

  EXPECT_THROW(factory.CreateGeometricEntity(
                   ffea::GeometricEntityType::kThirteenNodePiramid, {}),
               std::runtime_error);
}
