#include "../framework/inc/geometry/geometric_entity.h"

#include <gtest/gtest.h>

#include "../framework/inc/geometry/node.h"

TEST(TwoNodeLineTest, BasicAssertions) {
  ffea::Node node0(0, {0.0, 0.0, 0.0});
  ffea::Node node1(1, {1.0, 0.0, 0.0});

  const auto dim = 2;

  ffea::TwoNodeLine line(dim, {&node0, &node1});

  ASSERT_EQ(line.dim(), dim);
  ASSERT_EQ(line.type(), ffea::GeometricEntityType::kTwoNodeLine);
  ASSERT_EQ(line.number_of_nodes(), 2);
}
