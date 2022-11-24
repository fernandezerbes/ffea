#include "../framework/inc/geometry/geometric_entity.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../framework/inc/alias.h"
#include "../framework/inc/geometry/node.h"

using namespace testing;

class TwoNodeLineTest : public ::testing::Test {
 protected:
  TwoNodeLineTest()
      : dim(2),
        node0(24, {-1.2, -1.4, 0.0}),
        node1(78, {1.4, -2.0, 0.0}),
        line(dim, {&node0, &node1}) {}

  size_t dim;
  ffea::Node node0;
  ffea::Node node1;
  ffea::TwoNodeLine line;
};

TEST_F(TwoNodeLineTest, BasicExpectations) {
  ASSERT_EQ(line.type(), ffea::GeometricEntityType::kTwoNodeLine);
  ASSERT_EQ(line.dim(), dim);
  ASSERT_EQ(line.number_of_nodes(), 2);
  ASSERT_EQ(line.node_tag(0), 24);
  ASSERT_EQ(line.node_tag(1), 78);
}

TEST_F(TwoNodeLineTest, GlobalCoordinates) {
  EXPECT_NEAR(line.node_coords(0).get(0), -1.2, 1.e-6);
  EXPECT_NEAR(line.node_coords(0).get(1), -1.4, 1.e-6);
  EXPECT_NEAR(line.node_coords(0).get(2), 0.0, 1.e-6);
  EXPECT_NEAR(line.node_coords(1).get(0), 1.4, 1.e-6);
  EXPECT_NEAR(line.node_coords(1).get(1), -2.0, 1.e-6);
  EXPECT_NEAR(line.node_coords(1).get(2), 0.0, 1.e-6);
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

TEST_F(TwoNodeLineTest, ShapeFunctions) {
  // Middle of parametric domain
  ffea::Matrix<double> expected_N(1, 2);
  expected_N(0, 0) = 0.5;
  expected_N(0, 1) = 0.5;

  auto actual_N = line.EvaluateShapeFunctions({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 0
  expected_N(0, 0) = 1.0;
  expected_N(0, 1) = 0.0;

  actual_N = line.EvaluateShapeFunctions({-1.0, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 1
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 1.0;

  actual_N = line.EvaluateShapeFunctions({1.0, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At an interior point
  expected_N(0, 0) = 0.25;
  expected_N(0, 1) = 0.75;

  actual_N = line.EvaluateShapeFunctions({0.5, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));
}

TEST_F(TwoNodeLineTest, ShapeFunctionFirstDerivatives) {
  ffea::Matrix<double> expected_dN_local(1, 2);
  expected_dN_local(0, 0) = -0.5;
  expected_dN_local(0, 1) = 0.5;

  const auto actual_dN_local = line.EvaluateShapeFunctions(
      {0.0, 0.0, 0.0}, ffea::DerivativeOrder::kFirst);

  EXPECT_TRUE(actual_dN_local.isApprox(expected_dN_local));
}

TEST_F(TwoNodeLineTest, NormalVector) {
  ffea::Vector<double> expected_normal(2);
  expected_normal(0) = -0.3;
  expected_normal(1) = -1.3;

  const auto actual_normal = line.EvaluateNormalVector({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_normal.isApprox(expected_normal));
}

TEST_F(TwoNodeLineTest, Differential) {
  const auto expected_differential = 1.334166406;

  const auto actual_differential = line.EvaluateDifferential({0.0, 0.0, 0.0});

  EXPECT_NEAR(actual_differential, expected_differential, 1.e-6);
}

TEST_F(TwoNodeLineTest, MapLocalToGlobalCoordinates) {
  // Middle of parametric domain
  auto expected_coords = ffea::Coordinates(0.1, -1.7, 0.0);

  auto actual_coords = line.MapLocalToGlobal(ffea::Coordinates(0.0, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);

  // At node 0
  actual_coords = line.MapLocalToGlobal(ffea::Coordinates(-1.0, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node0.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node0.coords().get(1), 1.e-6);

  // At node 1
  actual_coords = line.MapLocalToGlobal(ffea::Coordinates(1.0, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node1.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node1.coords().get(1), 1.e-6);

  // At an interior point
  expected_coords = ffea::Coordinates(0.75, -1.85, 0.0);

  actual_coords = line.MapLocalToGlobal(ffea::Coordinates(0.5, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
}

class FourNodeQuad2DTest : public ::testing::Test {
 protected:
  FourNodeQuad2DTest()
      : dim(2),
        node0(24, {-1.0, 1.0, 0.0}),
        node1(78, {-1.0, -1.0, 0.0}),
        node2(94, {1.0, 0.0, 0.0}),
        node3(1, {1.0, 2.0, 0.0}),
        quad(dim, {&node0, &node1, &node2, &node3}) {}

  size_t dim;
  ffea::Node node0;
  ffea::Node node1;
  ffea::Node node2;
  ffea::Node node3;
  ffea::FourNodeQuad quad;
};

TEST_F(FourNodeQuad2DTest, BasicExpectations) {
  ASSERT_EQ(quad.type(), ffea::GeometricEntityType::kFourNodeQuad);
  ASSERT_EQ(quad.dim(), dim);
  ASSERT_EQ(quad.number_of_nodes(), 4);
  ASSERT_EQ(quad.node_tag(0), 24);
  ASSERT_EQ(quad.node_tag(1), 78);
  ASSERT_EQ(quad.node_tag(2), 94);
  ASSERT_EQ(quad.node_tag(3), 1);
}

TEST_F(FourNodeQuad2DTest, GlobalCoordinates) {
  EXPECT_NEAR(quad.node_coords(0).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(0).get(1), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(0).get(2), 0.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(1).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(1).get(1), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(1).get(2), 0.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(2).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(2).get(1), 0.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(2).get(2), 0.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(3).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(3).get(1), 2.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(3).get(2), 0.0, 1.e-6);
}

TEST_F(FourNodeQuad2DTest, NodalLocalCoordinates) {
  const auto nodal_local_coords = quad.nodal_local_coords();
  EXPECT_NEAR(nodal_local_coords[0].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(2), 0.0, 1.e-6);
}

TEST_F(FourNodeQuad2DTest, ShapeFunctions) {
  // Middle of parametric domain
  ffea::Matrix<double> expected_N(1, 4);
  expected_N(0, 0) = 0.25;
  expected_N(0, 1) = 0.25;
  expected_N(0, 2) = 0.25;
  expected_N(0, 3) = 0.25;

  auto actual_N = quad.EvaluateShapeFunctions({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 0
  expected_N(0, 0) = 1.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 0.0;

  actual_N = quad.EvaluateShapeFunctions({-1.0, -1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 1
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 1.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 0.0;

  actual_N = quad.EvaluateShapeFunctions({1.0, -1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 2
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 1.0;
  expected_N(0, 3) = 0.0;

  actual_N = quad.EvaluateShapeFunctions({1.0, 1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 3
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 1.0;

  actual_N = quad.EvaluateShapeFunctions({-1.0, 1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At an interior point r = 0.5, s = 0.5
  expected_N(0, 0) = 0.0625;
  expected_N(0, 1) = 0.1875;
  expected_N(0, 2) = 0.5625;
  expected_N(0, 3) = 0.1875;

  actual_N = quad.EvaluateShapeFunctions({0.5, 0.5, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));
}

TEST_F(FourNodeQuad2DTest, ShapeFunctionFirstDerivatives) {
  // At an interior point r = 0.5, s = 0.5
  ffea::Matrix<double> expected_dN_local(2, 4);
  expected_dN_local(0, 0) = -0.125;
  expected_dN_local(0, 1) = 0.125;
  expected_dN_local(0, 2) = 0.375;
  expected_dN_local(0, 3) = -0.375;
  expected_dN_local(1, 0) = -0.125;
  expected_dN_local(1, 1) = -0.375;
  expected_dN_local(1, 2) = 0.375;
  expected_dN_local(1, 3) = 0.125;

  const auto actual_dN_local = quad.EvaluateShapeFunctions(
      {0.5, 0.5, 0.0}, ffea::DerivativeOrder::kFirst);

  EXPECT_TRUE(actual_dN_local.isApprox(expected_dN_local));
}

TEST_F(FourNodeQuad2DTest, NormalVector) {
  ffea::Vector<double> expected_normal(3);
  expected_normal(0) = 0.0;
  expected_normal(1) = 0.0;
  expected_normal(2) = 1.0;

  const auto actual_normal = quad.EvaluateNormalVector({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_normal.isApprox(expected_normal));
}

TEST_F(FourNodeQuad2DTest, Differential) {
  const auto expected_differential = 1.0;

  const auto actual_differential = quad.EvaluateDifferential({0.0, 0.0, 0.0});

  EXPECT_NEAR(actual_differential, expected_differential, 1.e-6);
}

TEST_F(FourNodeQuad2DTest, MapLocalToGlobalCoordinates) {
  // Middle of parametric domain
  auto expected_coords = ffea::Coordinates(0.0, 0.5, 0.0);

  auto actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(0.0, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);

  // At node 0
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(-1.0, -1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node0.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node0.coords().get(1), 1.e-6);

  // At node 1
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(1.0, -1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node1.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node1.coords().get(1), 1.e-6);

  // At node 2
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(1.0, 1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node2.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node2.coords().get(1), 1.e-6);

  // At node 3
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(-1.0, 1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node3.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node3.coords().get(1), 1.e-6);

  // At an interior point
  expected_coords = ffea::Coordinates(0.5, 0.25, 0.0);

  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(0.5, 0.5, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
}

class FourNodeQuad3DTest : public ::testing::Test {
 protected:
  FourNodeQuad3DTest()
      : dim(3),
        node0(24, {-1.0, 1.0, -1.0}),
        node1(78, {-1.0, -1.0, -1.0}),
        node2(94, {1.0, 0.0, 1.0}),
        node3(1, {1.0, 2.0, 1.0}),
        quad(dim, {&node0, &node1, &node2, &node3}) {}

  size_t dim;
  ffea::Node node0;
  ffea::Node node1;
  ffea::Node node2;
  ffea::Node node3;
  ffea::FourNodeQuad quad;
};

TEST_F(FourNodeQuad3DTest, BasicExpectations) {
  ASSERT_EQ(quad.type(), ffea::GeometricEntityType::kFourNodeQuad);
  ASSERT_EQ(quad.dim(), dim);
  ASSERT_EQ(quad.number_of_nodes(), 4);
  ASSERT_EQ(quad.node_tag(0), 24);
  ASSERT_EQ(quad.node_tag(1), 78);
  ASSERT_EQ(quad.node_tag(2), 94);
  ASSERT_EQ(quad.node_tag(3), 1);
}

TEST_F(FourNodeQuad3DTest, GlobalCoordinates) {
  EXPECT_NEAR(quad.node_coords(0).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(0).get(1), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(0).get(2), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(1).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(1).get(1), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(1).get(2), -1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(2).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(2).get(1), 0.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(2).get(2), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(3).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(3).get(1), 2.0, 1.e-6);
  EXPECT_NEAR(quad.node_coords(3).get(2), 1.0, 1.e-6);
}

TEST_F(FourNodeQuad3DTest, NodalLocalCoordinates) {
  const auto nodal_local_coords = quad.nodal_local_coords();
  EXPECT_NEAR(nodal_local_coords[0].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(2), 0.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(2), 0.0, 1.e-6);
}

TEST_F(FourNodeQuad3DTest, ShapeFunctions) {
  // Middle of parametric domain
  ffea::Matrix<double> expected_N(1, 4);
  expected_N(0, 0) = 0.25;
  expected_N(0, 1) = 0.25;
  expected_N(0, 2) = 0.25;
  expected_N(0, 3) = 0.25;

  auto actual_N = quad.EvaluateShapeFunctions({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 0
  expected_N(0, 0) = 1.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 0.0;

  actual_N = quad.EvaluateShapeFunctions({-1.0, -1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 1
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 1.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 0.0;

  actual_N = quad.EvaluateShapeFunctions({1.0, -1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 2
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 1.0;
  expected_N(0, 3) = 0.0;

  actual_N = quad.EvaluateShapeFunctions({1.0, 1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 3
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 1.0;

  actual_N = quad.EvaluateShapeFunctions({-1.0, 1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At an interior point r = 0.5, s = 0.5
  expected_N(0, 0) = 0.0625;
  expected_N(0, 1) = 0.1875;
  expected_N(0, 2) = 0.5625;
  expected_N(0, 3) = 0.1875;

  actual_N = quad.EvaluateShapeFunctions({0.5, 0.5, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));
}

TEST_F(FourNodeQuad3DTest, ShapeFunctionFirstDerivatives) {
  // At an interior point r = 0.5, s = 0.5
  ffea::Matrix<double> expected_dN_local(2, 4);
  expected_dN_local(0, 0) = -0.125;
  expected_dN_local(0, 1) = 0.125;
  expected_dN_local(0, 2) = 0.375;
  expected_dN_local(0, 3) = -0.375;
  expected_dN_local(1, 0) = -0.125;
  expected_dN_local(1, 1) = -0.375;
  expected_dN_local(1, 2) = 0.375;
  expected_dN_local(1, 3) = 0.125;

  const auto actual_dN_local = quad.EvaluateShapeFunctions(
      {0.5, 0.5, 0.0}, ffea::DerivativeOrder::kFirst);

  EXPECT_TRUE(actual_dN_local.isApprox(expected_dN_local));
}

TEST_F(FourNodeQuad3DTest, NormalVector) {
  ffea::Vector<double> expected_normal(3);
  expected_normal(0) = -1.0;
  expected_normal(1) = 0.0;
  expected_normal(2) = 1.0;

  const auto actual_normal = quad.EvaluateNormalVector({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_normal.isApprox(expected_normal));
}

TEST_F(FourNodeQuad3DTest, Differential) {
  const auto expected_differential = std::sqrt(2);

  const auto actual_differential = quad.EvaluateDifferential({0.0, 0.0, 0.0});

  EXPECT_NEAR(actual_differential, expected_differential, 1.e-6);
}

TEST_F(FourNodeQuad3DTest, MapLocalToGlobalCoordinates) {
  // Middle of parametric domain
  auto expected_coords = ffea::Coordinates(0.0, 0.5, 0.0);

  auto actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(0.0, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), expected_coords.get(2), 1.e-6);

  // At node 0
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(-1.0, -1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node0.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node0.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node0.coords().get(2), 1.e-6);

  // At node 1
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(1.0, -1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node1.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node1.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node1.coords().get(2), 1.e-6);

  // At node 2
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(1.0, 1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node2.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node2.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node2.coords().get(2), 1.e-6);

  // At node 3
  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(-1.0, 1.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), node3.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node3.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node3.coords().get(2), 1.e-6);

  // At an interior point
  expected_coords = ffea::Coordinates(0.5, 0.25, 0.5);

  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(0.5, 0.5, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), expected_coords.get(2), 1.e-6);
}
