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

class Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const = 0;
};

class QuadratureRule1x2 : public Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const override;
};

class QuadratureRule2x2 : public Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const override;
};


class QuadratureRule2x2x2 : public Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const override;
};

class QuadratureRuleTria1 : public Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const override;
};

class QuadratureRuleTria3 : public Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const override;
};

class QuadratureRuleTetra1 : public Quadrature {
 public:
  virtual std::vector<IntegrationPoint> GetIntegrationPoints() const override;
};


}  // namespace ffea

#endif  // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
