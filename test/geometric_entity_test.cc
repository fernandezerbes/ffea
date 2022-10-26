#include "../framework/inc/geometry/geometric_entity.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../framework/inc/geometry/node.h"

using namespace testing;

class TwoNodeLineTest : public ::testing::Test {
 protected:

  TwoNodeLineTest()
      : dim(2),
        node0(24, {-1.2, -1.4, -2.8}),
        node1(78, {1.4, -2.0, 3.0}),
        line(dim, {&node0, &node1}) {}

  size_t dim;
  ffea::Node node0;
  ffea::Node node1;
  ffea::TwoNodeLine line;
};

TEST_F(TwoNodeLineTest, BasicExpectations) {
  EXPECT_EQ(line.type(), ffea::GeometricEntityType::kTwoNodeLine);
  EXPECT_EQ(line.dim(), dim);
  EXPECT_EQ(line.number_of_nodes(), 2);
  EXPECT_EQ(line.node_tag(0), 24);
  EXPECT_EQ(line.node_tag(1), 78);
}

TEST_F(TwoNodeLineTest, GlobalCoordinates) {
  EXPECT_NEAR(line.node_coords(0).get(0), -1.2, 1.e-6);
  EXPECT_NEAR(line.node_coords(0).get(1), -1.4, 1.e-6);
  EXPECT_NEAR(line.node_coords(0).get(2), -2.8, 1.e-6);
  EXPECT_NEAR(line.node_coords(1).get(0), 1.4, 1.e-6);
  EXPECT_NEAR(line.node_coords(1).get(1), -2.0, 1.e-6);
  EXPECT_NEAR(line.node_coords(1).get(2), 3.0, 1.e-6);
}

TEST_F(TwoNodeLineTest, NodalLocalCoordinates) {
  const auto nodal_local_coords = line.nodal_local_coords();
  EXPECT_NEAR(nodal_local_coords[0].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(1), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(1), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(2), 0.0, 1.e-6);
}

// TEST_F(TwoNodeLineTest, Differential) {
//   const auto line_length = 5.834380858;
//   const auto differential = line.EvaluateDifferential({0.0, 0.0, 0.0});

//   EXPECT_NEAR(differential, line_length, 1.e-6);
// }
