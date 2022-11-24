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

  // At an interior point r = 0.5
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

  // At an interior point r = 0.5
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

  actual_N = quad.EvaluateShapeFunctions({1.0, -1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 2
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 1.0;

  actual_N = quad.EvaluateShapeFunctions({1.0, 1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 3
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

  // At an interior point r = 0.5, s = 0.5
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

  actual_N = quad.EvaluateShapeFunctions({1.0, -1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 2
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 1.0;

  actual_N = quad.EvaluateShapeFunctions({1.0, 1.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 3
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

  // At an interior point r = 0.5, s = 0.5
  expected_coords = ffea::Coordinates(0.5, 0.25, 0.5);

  actual_coords = quad.MapLocalToGlobal(ffea::Coordinates(0.5, 0.5, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), expected_coords.get(2), 1.e-6);
}

class EightNodeHexTest : public ::testing::Test {
 protected:
  EightNodeHexTest()
      : dim(3),
        node0(24, {-1.0, 1.0, -1.0}),
        node1(78, {-1.0, -1.0, -1.0}),
        node2(94, {1.0, 0.0, -1.0}),
        node3(1, {1.0, 2.0, -1.0}),
        node4(23, {-1.0, 1.0, 1.0}),
        node5(2, {-1.0, -1.0, 1.0}),
        node6(14, {1.0, 0.0, 1.0}),
        node7(16, {1.0, 2.0, 1.0}),
        hex({&node0, &node1, &node2, &node3, &node4, &node5, &node6, &node7}) {
  }

  size_t dim;
  ffea::Node node0;
  ffea::Node node1;
  ffea::Node node2;
  ffea::Node node3;
  ffea::Node node4;
  ffea::Node node5;
  ffea::Node node6;
  ffea::Node node7;
  ffea::EightNodeHex hex;
};

TEST_F(EightNodeHexTest, BasicExpectations) {
  ASSERT_EQ(hex.type(), ffea::GeometricEntityType::kEightNodeHex);
  ASSERT_EQ(hex.dim(), dim);
  ASSERT_EQ(hex.number_of_nodes(), 8);
  ASSERT_EQ(hex.node_tag(0), 24);
  ASSERT_EQ(hex.node_tag(1), 78);
  ASSERT_EQ(hex.node_tag(2), 94);
  ASSERT_EQ(hex.node_tag(3), 1);
  ASSERT_EQ(hex.node_tag(4), 23);
  ASSERT_EQ(hex.node_tag(5), 2);
  ASSERT_EQ(hex.node_tag(6), 14);
  ASSERT_EQ(hex.node_tag(7), 16);
}

TEST_F(EightNodeHexTest, GlobalCoordinates) {
  EXPECT_NEAR(hex.node_coords(0).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(0).get(1), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(0).get(2), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(1).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(1).get(1), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(1).get(2), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(2).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(2).get(1), 0.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(2).get(2), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(3).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(3).get(1), 2.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(3).get(2), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(4).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(4).get(1), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(4).get(2), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(5).get(0), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(5).get(1), -1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(5).get(2), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(6).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(6).get(1), 0.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(6).get(2), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(7).get(0), 1.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(7).get(1), 2.0, 1.e-6);
  EXPECT_NEAR(hex.node_coords(7).get(2), 1.0, 1.e-6);
}

TEST_F(EightNodeHexTest, NodalLocalCoordinates) {
  const auto nodal_local_coords = hex.nodal_local_coords();
  EXPECT_NEAR(nodal_local_coords[0].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[0].get(2), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[1].get(2), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[2].get(2), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[3].get(2), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[4].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[4].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[4].get(2), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[5].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[5].get(1), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[5].get(2), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[6].get(0), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[6].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[6].get(2), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[7].get(0), -1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[7].get(1), 1.0, 1.e-6);
  EXPECT_NEAR(nodal_local_coords[7].get(2), 1.0, 1.e-6);
}

TEST_F(EightNodeHexTest, ShapeFunctions) {
  // Middle of parametric domain
  ffea::Matrix<double> expected_N(1, 8);
  expected_N(0, 0) = 0.125;
  expected_N(0, 1) = 0.125;
  expected_N(0, 2) = 0.125;
  expected_N(0, 3) = 0.125;
  expected_N(0, 4) = 0.125;
  expected_N(0, 5) = 0.125;
  expected_N(0, 6) = 0.125;
  expected_N(0, 7) = 0.125;

  auto actual_N = hex.EvaluateShapeFunctions({0.0, 0.0, 0.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 0
  expected_N(0, 0) = 1.0;
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 0.0;
  expected_N(0, 4) = 0.0;
  expected_N(0, 5) = 0.0;
  expected_N(0, 6) = 0.0;
  expected_N(0, 7) = 0.0;

  actual_N = hex.EvaluateShapeFunctions({-1.0, -1.0, -1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 1
  expected_N(0, 0) = 0.0;
  expected_N(0, 1) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({1.0, -1.0, -1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 2
  expected_N(0, 1) = 0.0;
  expected_N(0, 2) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({1.0, 1.0, -1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 3
  expected_N(0, 2) = 0.0;
  expected_N(0, 3) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({-1.0, 1.0, -1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 4
  expected_N(0, 3) = 0.0;
  expected_N(0, 4) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({-1.0, -1.0, 1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 5
  expected_N(0, 4) = 0.0;
  expected_N(0, 5) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({1.0, -1.0, 1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 6
  expected_N(0, 5) = 0.0;
  expected_N(0, 6) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({1.0, 1.0, 1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At node 7
  expected_N(0, 6) = 0.0;
  expected_N(0, 7) = 1.0;

  actual_N = hex.EvaluateShapeFunctions({-1.0, 1.0, 1.0});

  EXPECT_TRUE(actual_N.isApprox(expected_N));

  // At an interior point r = 0.5, s = 0.5, t = 0.5
  expected_N(0, 0) = 0.015625;
  expected_N(0, 1) = 0.046875;
  expected_N(0, 2) = 0.140625;
  expected_N(0, 3) = 0.046875;
  expected_N(0, 4) = 0.046875;
  expected_N(0, 5) = 0.140625;
  expected_N(0, 6) = 0.421875;
  expected_N(0, 7) = 0.140625;

  actual_N = hex.EvaluateShapeFunctions({0.5, 0.5, 0.5});

  EXPECT_TRUE(actual_N.isApprox(expected_N));
}

TEST_F(EightNodeHexTest, ShapeFunctionFirstDerivatives) {
  // At an interior point r = 0.5, s = 0.5, t = 0.5
  ffea::Matrix<double> expected_dN_local(3, 8);
  expected_dN_local(0, 0) = -0.03125;
  expected_dN_local(0, 1) = 0.03125;
  expected_dN_local(0, 2) = 0.09375;
  expected_dN_local(0, 3) = -0.09375;
  expected_dN_local(0, 4) = -0.09375;
  expected_dN_local(0, 5) = 0.09375;
  expected_dN_local(0, 6) = 0.28125;
  expected_dN_local(0, 7) = -0.28125;

  expected_dN_local(1, 0) = -0.03125;
  expected_dN_local(1, 1) = -0.09375;
  expected_dN_local(1, 2) = 0.09375;
  expected_dN_local(1, 3) = 0.03125;
  expected_dN_local(1, 4) = -0.09375;
  expected_dN_local(1, 5) = -0.28125;
  expected_dN_local(1, 6) = 0.28125;
  expected_dN_local(1, 7) = 0.09375;

  expected_dN_local(2, 0) = -0.03125;
  expected_dN_local(2, 1) = -0.09375;
  expected_dN_local(2, 2) = -0.28125;
  expected_dN_local(2, 3) = -0.09375;
  expected_dN_local(2, 4) = 0.03125;
  expected_dN_local(2, 5) = 0.09375;
  expected_dN_local(2, 6) = 0.28125;
  expected_dN_local(2, 7) = 0.09375;

  const auto actual_dN_local = hex.EvaluateShapeFunctions(
      {0.5, 0.5, 0.5}, ffea::DerivativeOrder::kFirst);

  EXPECT_TRUE(actual_dN_local.isApprox(expected_dN_local));
}

TEST_F(EightNodeHexTest, NormalVector) {
  ASSERT_THROW(hex.EvaluateNormalVector({0.0, 0.0, 0.0}), std::logic_error);
}

TEST_F(EightNodeHexTest, Differential) {
  const auto expected_differential = 1.0;

  const auto actual_differential = hex.EvaluateDifferential({0.0, 0.0, 0.0});

  EXPECT_NEAR(actual_differential, expected_differential, 1.e-6);
}

TEST_F(EightNodeHexTest, MapLocalToGlobalCoordinates) {
  // Middle of parametric domain
  auto expected_coords = ffea::Coordinates(0.0, 0.5, 0.0);

  auto actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(0.0, 0.0, 0.0));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), expected_coords.get(2), 1.e-6);

  // At node 0
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(-1.0, -1.0, -1.0));

  EXPECT_NEAR(actual_coords.get(0), node0.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node0.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node0.coords().get(2), 1.e-6);

  // At node 1
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(1.0, -1.0, -1.0));

  EXPECT_NEAR(actual_coords.get(0), node1.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node1.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node1.coords().get(2), 1.e-6);

  // At node 2
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(1.0, 1.0, -1.0));

  EXPECT_NEAR(actual_coords.get(0), node2.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node2.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node2.coords().get(2), 1.e-6);

  // At node 3
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(-1.0, 1.0, -1.0));

  EXPECT_NEAR(actual_coords.get(0), node3.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node3.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node3.coords().get(2), 1.e-6);

  // At node 4
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(-1.0, -1.0, 1.0));

  EXPECT_NEAR(actual_coords.get(0), node4.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node4.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node4.coords().get(2), 1.e-6);

  // At node 5
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(1.0, -1.0, 1.0));

  EXPECT_NEAR(actual_coords.get(0), node5.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node5.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node5.coords().get(2), 1.e-6);

  // At node 6
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(1.0, 1.0, 1.0));

  EXPECT_NEAR(actual_coords.get(0), node6.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node6.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node6.coords().get(2), 1.e-6);

  // At node 7
  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(-1.0, 1.0, 1.0));

  EXPECT_NEAR(actual_coords.get(0), node7.coords().get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), node7.coords().get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), node7.coords().get(2), 1.e-6);

  // At an interior point r = 0.5, s = 0.5, t = 0.5
  expected_coords = ffea::Coordinates(0.5, 0.25, 0.5);

  actual_coords = hex.MapLocalToGlobal(ffea::Coordinates(0.5, 0.5, 0.5));

  EXPECT_NEAR(actual_coords.get(0), expected_coords.get(0), 1.e-6);
  EXPECT_NEAR(actual_coords.get(1), expected_coords.get(1), 1.e-6);
  EXPECT_NEAR(actual_coords.get(2), expected_coords.get(2), 1.e-6);
}
