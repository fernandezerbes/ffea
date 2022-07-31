#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_

#include <memory>
#include <vector>

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

using IntegrationPointsGroupPtr =
    std::shared_ptr<std::vector<IntegrationPoint>>;

class IntegrationRule {
 public:
  virtual IntegrationPointsGroupPtr GetIntegrationPoints() const = 0;
};

class Integration1x2Rule : public IntegrationRule {
 public:
  virtual IntegrationPointsGroupPtr GetIntegrationPoints() const override;
};

class Integration2x2Rule : public IntegrationRule {
 public:
  virtual IntegrationPointsGroupPtr GetIntegrationPoints() const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
