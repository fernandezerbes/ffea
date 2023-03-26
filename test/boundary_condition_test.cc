#include "../framework/inc/model/boundary_condition.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "../framework/inc/alias.h"
#include "../framework/inc/geometry/geometric_entity.h"
#include "../framework/inc/geometry/geometric_entity_factory.h"
#include "../framework/inc/geometry/node.h"
#include "../framework/inc/mesh/mesh.h"

using namespace testing;

class DirectEnforcementStrategyTest : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    // Set up stiffness matrix
    stiffness = ffea::CSRMatrix<double>(6, 6);
    ffea::MatrixEntries<double> nonzero_entries(36);
    nonzero_entries.emplace_back(0, 0, 20);
    nonzero_entries.emplace_back(0, 1, 10);
    nonzero_entries.emplace_back(0, 2, -10);
    nonzero_entries.emplace_back(0, 3, 0);
    nonzero_entries.emplace_back(0, 4, -10);
    nonzero_entries.emplace_back(0, 5, -10);

    nonzero_entries.emplace_back(1, 0, 10);
    nonzero_entries.emplace_back(1, 1, 10);
    nonzero_entries.emplace_back(1, 2, 0);
    nonzero_entries.emplace_back(1, 3, 0);
    nonzero_entries.emplace_back(1, 4, -10);
    nonzero_entries.emplace_back(1, 5, -10);

    nonzero_entries.emplace_back(2, 0, -10);
    nonzero_entries.emplace_back(2, 1, 0);
    nonzero_entries.emplace_back(2, 2, 10);
    nonzero_entries.emplace_back(2, 3, 0);
    nonzero_entries.emplace_back(2, 4, 0);
    nonzero_entries.emplace_back(2, 5, 0);

    nonzero_entries.emplace_back(3, 0, 0);
    nonzero_entries.emplace_back(3, 1, 0);
    nonzero_entries.emplace_back(3, 2, 0);
    nonzero_entries.emplace_back(3, 3, 5);
    nonzero_entries.emplace_back(3, 4, 0);
    nonzero_entries.emplace_back(3, 5, -5);

    nonzero_entries.emplace_back(4, 0, -10);
    nonzero_entries.emplace_back(4, 1, -10);
    nonzero_entries.emplace_back(4, 2, 0);
    nonzero_entries.emplace_back(4, 3, 0);
    nonzero_entries.emplace_back(4, 4, 10);
    nonzero_entries.emplace_back(4, 5, 10);

    nonzero_entries.emplace_back(5, 0, -10);
    nonzero_entries.emplace_back(5, 1, -10);
    nonzero_entries.emplace_back(5, 2, 0);
    nonzero_entries.emplace_back(5, 3, -5);
    nonzero_entries.emplace_back(5, 4, 10);
    nonzero_entries.emplace_back(5, 5, 15);
    stiffness.setFromTriplets(nonzero_entries.begin(), nonzero_entries.end());

    // Set up rhs vector
    rhs = ffea::Vector<double>(6);
    rhs(0) = 0;
    rhs(1) = 0;
    rhs(2) = 0;
    rhs(3) = 0;
    rhs(4) = 2;
    rhs(5) = 1;

    // Set up nodes
    nodes.push_back(ffea::Node(0, {0.0, 0.0, 0.0}));
    nodes.push_back(ffea::Node(1, {10.0, 0.0, 0.0}));
    nodes.push_back(ffea::Node(2, {10.0, 10.0, 0.0}));

    // Set up dofs
    dofs.emplace_back(0);
    dofs.emplace_back(1);
    dofs.emplace_back(2);
    dofs.emplace_back(3);
    dofs.emplace_back(4);
    dofs.emplace_back(5);

    // Set up geometric entities (points)
    ffea::GeometricEntityFactory2D entity_factory;
    for (auto& node : nodes) {
      points.push_back(
          entity_factory.CreateGeometricEntity(ffea::GeometricEntityType::kOneNodePoint, {&node}));
    }

    // Set up elements (points)
    entity_factory = ffea::GeometricEntityFactory2D();
    ffea::ElementFactory element_factory(ffea::full_integration_points);
    elements.push_back(element_factory.CreateElement(*points[0], {&dofs[0], &dofs[1]}));
    elements.push_back(element_factory.CreateElement(*points[1], {&dofs[2], &dofs[3]}));

    // Set up
    displacement_function = [](const ffea::Coordinates& coords,
                               ffea::Time t) -> std::vector<double> {
      if (coords.get(0) == 0.0 && coords.get(1) == 0.0) {
        return {0, -0.5};
      }

      if (coords.get(0) == 10.0 && coords.get(1) == 0.0) {
        return {0, 0.4};
      }

      return {0, 0};
    };
  }

  ffea::CSRMatrix<double> stiffness;
  ffea::Vector<double> rhs;
  std::vector<ffea::Node> nodes;
  std::vector<ffea::DegreeOfFreedom> dofs;
  std::vector<std::unique_ptr<ffea::GeometricEntity>> points;
  std::vector<ffea::Element> elements;
  ffea::SpatioTemporalFunction<std::vector<double>> displacement_function;
};

TEST_F(DirectEnforcementStrategyTest, Enforcement) {
  auto enforcement_strategy = ffea::DirectEnforcementStrategy();
  enforcement_strategy.Enforce(stiffness, rhs, displacement_function, elements, {0, 1}, 0.0);

  ffea::Matrix<double> expected_stiffness = ffea::Matrix<double>::Identity(6, 6);
  expected_stiffness(4, 4) = 10;
  expected_stiffness(4, 5) = 10;
  expected_stiffness(5, 4) = 10;
  expected_stiffness(5, 5) = 15;

  ffea::Vector<double> expected_rhs = ffea::Vector<double>::Zero(6);
  expected_rhs(1) = -0.5;
  expected_rhs(3) = 0.4;
  expected_rhs(4) = -3;
  expected_rhs(5) = -2;

  for (size_t row = 0; row < stiffness.outerSize(); row++) {
    for (size_t col = 0; col < stiffness.innerSize(); col++) {
      EXPECT_DOUBLE_EQ(stiffness.coeff(row, col), expected_stiffness(row, col));
    }
    EXPECT_DOUBLE_EQ(rhs(row), expected_rhs(row));
  }
}
